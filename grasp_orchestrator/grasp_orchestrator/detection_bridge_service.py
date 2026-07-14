import json
import os
import tempfile
import time
from pathlib import Path
from typing import Dict

import rclpy
from geometry_msgs.msg import PoseStamped
from grasp_orchestrator_interfaces.srv import DetectGraspPose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .pose_utils import pose_from_json


class DetectionBridgeService(Node):
    def __init__(self) -> None:
        super().__init__("detection_bridge_service")
        default_save_dir = os.environ.get(
            "GRASP_RUNTIME_DIR", str(Path(tempfile.gettempdir()) / "graspness")
        )
        self.declare_parameter("service_name", "/detect_grasp_pose")
        self.declare_parameter("result_topic", "/grasp_detection/grasp_pose")
        self.declare_parameter("input_mode", "basic")
        self.declare_parameter(
            "color_topic", "/hdas/camera_wrist_right/color/image_raw"
        )
        self.declare_parameter(
            "depth_topic",
            "/hdas/camera_wrist_right/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "camera_info_topic",
            "/hdas/camera_wrist_right/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter(
            "scene_topic", "/perception/task1/rest_point_cloud"
        )
        self.declare_parameter(
            "target_topic", "/perception/task1/target_point_cloud"
        )
        self.declare_parameter("save_dir", default_save_dir)
        self.declare_parameter("source_frame_override", "")
        self.declare_parameter("default_timeout_sec", 20.0)
        self.declare_parameter("input_ready_timeout_sec", 2.0)
        self.declare_parameter("poll_interval_sec", 0.1)
        self.declare_parameter("cleanup_stale_result", True)
        self.declare_parameter("default_voxel_size", 0.002)
        self.declare_parameter("default_num_point", 50000)
        self.declare_parameter("default_collision_thresh", 0.03)
        self.declare_parameter("default_angle_cos_thr", 0.85)
        self.declare_parameter("default_max_width", 0.10)
        self.declare_parameter("default_min_width", 0.06)
        self.declare_parameter("default_topk", 100)
        # The right wrist camera sees the gripper in the near field. Keep that
        # region in the diagnostic cloud but exclude it from grasp inference.
        self.declare_parameter("default_z_min", 0.25)
        self.declare_parameter("default_z_max", 0.7)

        service_name = self.get_parameter("service_name").value
        result_topic = self.get_parameter("result_topic").value
        self.input_mode = str(self.get_parameter("input_mode").value).strip().lower()
        if self.input_mode not in ("basic", "atec"):
            raise ValueError(
                f"unsupported input_mode '{self.input_mode}', expected basic or atec"
            )
        basic_topics = {
            "color": str(self.get_parameter("color_topic").value),
            "aligned_depth": str(self.get_parameter("depth_topic").value),
            "camera_info": str(self.get_parameter("camera_info_topic").value),
        }
        atec_topics = {
            "scene": str(self.get_parameter("scene_topic").value),
            "target": str(self.get_parameter("target_topic").value),
        }
        self.input_topics = basic_topics if self.input_mode == "basic" else atec_topics

        self.save_dir = Path(
            self.get_parameter("save_dir").get_parameter_value().string_value
        )
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.trigger_path = self.save_dir / "trigger_detection.flag"
        self.result_path = self.save_dir / "latest_grasp.json"
        self.config_path = self.save_dir / "detection_config.json"

        self.callback_group = ReentrantCallbackGroup()
        self.pose_pub = self.create_publisher(PoseStamped, result_topic, 10)
        self.service = self.create_service(
            DetectGraspPose,
            service_name,
            self.handle_detect,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            f"detection bridge ready: service={service_name} "
            f"mode={self.input_mode} inputs={list(self.input_topics.values())} "
            f"save_dir={self.save_dir}"
        )

    def build_config(self) -> Dict[str, object]:
        return {
            "voxel_size": float(self.get_parameter("default_voxel_size").value),
            "num_point": int(self.get_parameter("default_num_point").value),
            "collision_thresh": float(self.get_parameter("default_collision_thresh").value),
            "angle_cos_thr": float(self.get_parameter("default_angle_cos_thr").value),
            "max_width": float(self.get_parameter("default_max_width").value),
            "min_width": float(self.get_parameter("default_min_width").value),
            "topk": int(self.get_parameter("default_topk").value),
            "z_min": float(self.get_parameter("default_z_min").value),
            "z_max": float(self.get_parameter("default_z_max").value),
        }

    def resolve_source_frame(self, result: Dict[str, object]) -> str:
        source_frame_override = str(
            self.get_parameter("source_frame_override").value
        ).strip().lstrip("/")
        if source_frame_override:
            return source_frame_override
        result_frame = str(result.get("source_frame", "")).strip().lstrip("/")
        if result_frame:
            return result_frame
        return "hdas/camera_wrist_right_color_optical_frame"

    def wait_for_input_publishers(self):
        timeout_sec = float(self.get_parameter("input_ready_timeout_sec").value)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            missing = [
                topic
                for topic in self.input_topics.values()
                if self.count_publishers(topic) == 0
            ]
            if not missing:
                return []
            time.sleep(0.05)
        return [
            topic
            for topic in self.input_topics.values()
            if self.count_publishers(topic) == 0
        ]

    def handle_detect(self, request: DetectGraspPose.Request, response: DetectGraspPose.Response):
        self.get_logger().info(
            f"received detection request target_frame='{request.target_frame}'"
        )
        timeout_sec = request.timeout_sec
        if timeout_sec <= 0.0:
            timeout_sec = float(self.get_parameter("default_timeout_sec").value)
        poll_interval = float(self.get_parameter("poll_interval_sec").value)
        cleanup_stale = bool(self.get_parameter("cleanup_stale_result").value)

        missing_topics = self.wait_for_input_publishers()
        if missing_topics:
            response.success = False
            response.message = (
                f"{self.input_mode} input is incomplete; missing publishers: "
                + ", ".join(missing_topics)
            )
            self.get_logger().error(response.message)
            return response

        if cleanup_stale:
            self.result_path.unlink(missing_ok=True)
        self.trigger_path.unlink(missing_ok=True)

        config = self.build_config()
        self.config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
        self.trigger_path.write_text("trigger\n", encoding="utf-8")
        self.get_logger().info("detection trigger published")

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            if self.result_path.exists() and not self.trigger_path.exists():
                break
            time.sleep(poll_interval)

        if not self.result_path.exists():
            self.trigger_path.unlink(missing_ok=True)
            response.success = False
            response.message = (
                f"timeout waiting for grasp result in {timeout_sec:.1f}s, "
                f"check the {self.input_mode} detector input topics"
            )
            self.get_logger().error(response.message)
            return response

        self.get_logger().info("detection result file received")

        try:
            result = json.loads(self.result_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            response.success = False
            response.message = f"invalid grasp result json: {exc}"
            self.get_logger().error(response.message)
            return response

        if not bool(result.get("success", False)):
            response.success = False
            response.message = str(result.get("message", "grasp detection failed"))
            return response

        source_frame = self.resolve_source_frame(result)
        self.get_logger().info(f"resolved source frame='{source_frame}'")
        pose = pose_from_json(source_frame, self.get_clock().now().to_msg(), result)
        self.get_logger().info("camera-frame pose assembled")
        requested_target = request.target_frame.strip().lstrip("/")
        if requested_target:
            self.get_logger().debug(
                f"ignoring DetectGraspPose.target_frame={requested_target}; "
                "returning camera-frame pose"
            )

        self.pose_pub.publish(pose)
        self.get_logger().info("camera-frame pose published")
        response.success = True
        response.message = "grasp detected"
        response.grasp_pose = pose
        response.score = float(result.get("score", 0.0))
        response.width = float(result.get("width", 0.0))
        response.height = float(result.get("height", 0.0))
        response.depth = float(result.get("depth", 0.0))
        response.object_id = int(result.get("object_id", 0))
        response.source_frame = source_frame
        self.get_logger().info(
            f"returning camera-frame grasp pose frame='{source_frame}'"
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = DetectionBridgeService()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
