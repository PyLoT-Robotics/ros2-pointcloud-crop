# ROS 2 PointCloud Crop

English · [日本語](docs/README.ja.md)

A standalone package for cropping PointCloud2 data with multiple boxes and
removing robot self-returns or nearby points. It reuses PyLoT's box geometry
and generalizes the point-record layout handling.

- Remove the union of boxes, or keep only points inside that union.
- Define boxes in another frame and evaluate them using TF at the cloud's acquisition time.
- Support FLOAT32/FLOAT64 XYZ, big/little endian and padded rows.
- Preserve every byte and field definition of retained points, including intensity, timestamps and rings.
- Validate each message's layout instead of reusing offsets from an earlier cloud.

## Usage

```bash
ros2 run pointcloud_crop pointcloud_crop_node --ros-args \
  -p input_topic:=/points -p output_topic:=/points/cropped \
  -p box_frame:=base_link -p boxes:='[-0.5,0.5,-0.4,0.4,-0.2,1.0]'
```

`boxes` contains `[min_x,max_x,min_y,max_y,min_z,max_z]` for each box, in metres.
Set `remove_inside:=false` to retain only inside points. An empty `box_frame`
uses the input cloud frame. Output is a compact cloud with height 1; input row
padding is removed. Coordinates and `header.frame_id` remain in the input frame:
TF is used only to evaluate the crop. NaN coordinates are retained in self-removal
mode and removed in inside-only mode.

Missing TF and malformed layouts cause the message to be dropped with a warning,
changing the original implementation's pass-through behavior. When TF is needed,
supply a valid acquisition timestamp rather than relying on stamp=0 and latest TF.

Process an Nx3 `.npy` array without ROS, in its existing coordinate frame:

```bash
ros2-pointcloud-crop crop points.npy cropped.npy --boxes -1 1 -1 1 -1 1
```

Tests cover transforms, multiple boxes, NaNs, row padding, extra fields, both
byte orders and FLOAT64. A ROS integration test uses synthetic TF and a cloud
to verify that coordinates and intensity are preserved.

## Installation and checks

Requires Python 3.10+. Run these commands from the repository root.
The core works in a regular Python environment without ROS.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m pointcloud_crop.cli demo
```

For ROS 2 Jazzy, place the repository in a regular workspace's `src` directory
and build it in a ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select pointcloud_crop
source install/setup.bash
```

Adapt `config/params.yaml` to your inputs and outputs. Parameters are read at startup.
Start the node with
`ros2 launch pointcloud_crop pointcloud_crop.launch.py params_file:=/absolute/path/to/params.yaml`.
CI runs core tests and the synthetic demo on Python 3.10 and 3.12. ROS integration
tests run when ROS is available and are skipped otherwise. These checks do not
certify model, physical-sensor or real-arm performance.

## License and source

This repository is currently private. Apache-2.0. See
[PROVENANCE.md](PROVENANCE.md) for the source commit, files and extraction changes.
There is no dependency on the original monorepo.
