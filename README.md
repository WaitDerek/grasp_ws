# Grasp pose detection workspace

ROS 2 Humble workspace for one-shot grasp pose detection from synchronized D405
color and aligned-depth images. Robot initialization and motion execution belong
to `mission_ws`; this workspace only provides `/detect_grasp_pose`.

Two detector modes are available:

- `basic` (default): consumes the complete RGB-D image and builds a full scene
  cloud internally.
- `atec`: consumes an upstream target-object cloud and a rest-of-scene cloud;
  inference uses the target while collision checking uses both clouds.

## Clone

The Git repository is the workspace `src` directory:

```bash
mkdir -p grasp_ws
git clone --recurse-submodules \
  git@github.com:WaitDerek/grasp_ws.git grasp_ws/src
cd grasp_ws
export GRASP_WS="$PWD"
```

For an existing clone, initialize the detector submodule with:

```bash
git -C "$GRASP_WS/src" submodule update --init --recursive
```

## Install

Use the `changan` Conda environment. Install a CUDA-enabled PyTorch build that
matches the local CUDA toolkit first. Install the normal dependencies, then
install graspnetAPI without its obsolete NumPy pins. Build MinkowskiEngine 0.5.4
from source as documented in `requirements.txt`.

```bash
conda activate changan
python -c "import torch; print(torch.__version__, torch.version.cuda)"
python -m pip install -r "$GRASP_WS/src/graspness_c/requirements.txt"
python -m pip install --no-deps graspnetAPI==1.2.11
python -m pip install --no-build-isolation -v \
  "$GRASP_WS/src/graspness_c/pointnet2"
python -m pip install --no-build-isolation -v \
  "$GRASP_WS/src/graspness_c/knn"
```

Download the RealSense checkpoint documented in
`graspness_c/README.md` and place it at:

```text
src/graspness_c/logs/log_kn/minkuresunet_realsense.tar
```

Build from the workspace root:

```bash
conda activate changan
source /opt/ros/humble/setup.zsh
cd "$GRASP_WS"
colcon build --merge-install --symlink-install \
  --cmake-args "-DCMAKE_BUILD_TYPE=Release" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Start

Set `GRASP_WS` to the workspace root in each new terminal.

If the D405 is not already publishing aligned depth, start it in terminal 1:

```bash
source /opt/ros/humble/setup.zsh
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=d405 \
  align_depth.enable:=true \
  enable_sync:=true
```

Start detection in terminal 2:

```bash
conda activate changan
source /opt/ros/humble/setup.zsh
source "$GRASP_WS/install/setup.zsh"
ros2 launch grasp_orchestrator grasp_detection.launch.py
```

To use the target-object ATEC pipeline instead:

```bash
ros2 launch grasp_orchestrator grasp_detection.launch.py \
  detector_mode:=atec \
  scene_topic:=/perception/task1/rest_point_cloud \
  target_topic:=/perception/task1/target_point_cloud
```

For the basic RGB point-cloud view with 10 grippers, use:

```bash
ros2 launch grasp_orchestrator grasp_detection.launch.py \
  visualize:=true visualization_grasps:=10
```

Call the service from terminal 3:

```bash
conda activate changan
source /opt/ros/humble/setup.zsh
source "$GRASP_WS/install/setup.zsh"
ros2 service call /detect_grasp_pose \
  grasp_orchestrator_interfaces/srv/DetectGraspPose \
  "{target_frame: '', target_label: 0, timeout_sec: 20.0}"
```

An empty `target_frame` returns a pose in `d405_color_optical_frame`. Use a
robot frame such as `torso_link` only when that TF is available.

Run only one detection launch for a runtime directory. The default
`mission_controller mission.launch.py` already starts perception; do not also
start this launch separately, or start mission_ws with `start_perception:=false`.
