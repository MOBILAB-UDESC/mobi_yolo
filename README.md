# Mobi_yolo
## Description
Ros2 package for real-time object detection and localization using YOLO and RGBD cameras.

## Status
**In Progress** - Basic functionality implemented

## Repository Structure
```tree
mobi_yolo/
├── yolo_detection/
├── README.md
└── requirements.txt
```

## Setting up the workspace
``` bash
mkdir ~/mobi_yolo && cd ~/mobi_yolo
python3 -m venv ./venv
source ./venv/bin/activate
pip install -r requirements.txt
touch ./venv/COLCON_IGNORE
git clone https://github.com/MOBILAB-UDESC/mobi_robots.git
rosdep install --from-paths mobi_yolo --ignore-src -r -y
colcon build
source install/setup.bash
```

## Usage
```bash
ros2 launch yolo_detection detection.launch.py rviz:=true rgb:=false
```
### Launch Arguments
| Argument | Description | Default | Options |
|----------|-------------|---------|---------|
| `rviz` | Whether to execute rviz2. | `false` | `true/false` |
| `rgb` | 📷 Enable opencv visualization for rgb image | `false` | `true/false` |
| `depth` | 📷 Enable opencv visualization for depth image as colormap | `false` | `true/false` |
| `detection_config` | 📄 Full path to the detection parameters file to use for the node | [detection.yaml](https://github.com/MOBILAB-UDESC/mobi_yolo/tree/main/yolo_detection/config/detection.yaml) | Custom config |
| `use_sim_time` | ⏰ Whether to use simulation time | `false` | `true/false` |

## Configuration file
Parameters used in the detection_node which can be found in config/detection.yaml.

<details>

  <summary>
    TF parameters
  </summary>

- **camera_frame**
  - RGB camera's reference frame for local coordinates of objects.
  - Type: str
  - Default value: 'camera_link'
- **base_frame**
  - Target frame for global object position (robot base)coordinates of objects.
  - Type: str
  - Default value: 'base_link'

</details>

<details>

  <summary>
    Camera parameters
  </summary>

- **camera_info**
  - Camera intrinsics (K matrix)
  - Type: str
  - Default value: 'camera/camera/aligned_depth_to_color/camera_info'
- **depth_topic**
  - Aligned depth image
  - Type: str
  - Default value: 'camera/camera/aligned_depth_to_color/image_raw'
- **rgb_topic**
  - RGB image
  - Type: str
  - Default value: 'camera/camera/color/image_raw'
- **depth_scale**
  - Scale factor to convert depth image to meters (depth_image / depth_scale)
  - Type: double
  - Default value: 1.0

</details>

<details>

  <summary>
    Synchronization parameters
  </summary>

- **queue_size**
  - Message buffer size for syncing depth and RGB images
  - Type: int
  - Default value: 10
- **sync_slop**
  - Max time difference (s)
  - Type: double
  - Default value: 0.1

</details>

<details>

  <summary>
    OpenCV parameters
  </summary>

- **visualize_rgb_image**
  - Enable opencv visualization for RGB image
  - Can be set via launch argument: `rgb:=true/false`
  - Type: bool
  - Default value: True
- **visualize_depth_image**
  - Enable opencv visualization for depth image
  - Can be set via launch argument: `depth:=true/false`
  - Type: bool
  - Default value: False

</details>

<details>

  <summary>
    Detection parameters
  </summary>

- **model_path**
  - Path to the YOLO model.
  - Type: str
  - Default value: 'mobi_yolo/yolo_detection/weights/apple.pt'
- **conf**
  - Minimum confidence threshold for detections.
  - Type: double
  - Default value: 0.5
- **iou**
  - Intersection Over Union threshold for Non-Maximum Suppression.
  - Type: double
  - Default value: 0.7
- **publish_poses**
  - Publish RGB image with bounding boxes in /detection/image topic
  - Type: bool
  - Default value: True
- **sphere_shape**
  - Use sphere shape for 3D bounding box
  - Type: bool
  - Default value: False

</details>

## First test with RGBD camera on Gazebo Harmonic
Robot base_link position: z = 0.06m relative to gazebo world. A world link at z = 0.0 is needed for proper global localization.

### Ground Truth (Gazebo)
<img src="https://raw.githubusercontent.com/MOBILAB-UDESC/mobi_yolo/main/gazebo_apple.png" alt="Apple position on Gazebo" width="900"/>

### Detection Result
<img src="https://raw.githubusercontent.com/MOBILAB-UDESC/mobi_yolo/main/opencv_apple.png" alt="Apple position estimated by the node" width="900"/>|