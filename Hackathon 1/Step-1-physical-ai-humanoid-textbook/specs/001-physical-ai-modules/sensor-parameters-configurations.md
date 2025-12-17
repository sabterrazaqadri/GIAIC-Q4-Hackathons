# Sensor Parameters and Configurations for Humanoid Robots

## Overview
This document details the key parameters for configuring sensors in humanoid robot simulations. Proper configuration of these parameters is essential for realistic sensor simulation and effective perception system development.

## LiDAR Sensor Parameters

### Key Parameters
- **Type**: 2D or 3D LiDAR
- **Range**: 
  - Min range: Typically 0.1m to 0.3m
  - Max range: 10m to 100m depending on sensor
- **Field of View (FOV)**:
  - Horizontal: 360° for spinning LiDARs, narrower for solid-state
  - Vertical: Varies from 20° to 45° for 3D LiDARs
- **Resolution**:
  - Angular resolution (horizontal): 0.1° to 1.0°
  - Channels (vertical) for 3D: 16, 32, 64, 128, etc.
- **Scan Rate**: 5-20 Hz for most sensors
- **Update Rate**: How frequently data is published

### Example Configuration (Velodyne VLP-16)
```yaml
lidar:
  type: "Velodyne_VLP_16"
  range:
    min: 0.1
    max: 100.0
  fov:
    horizontal: 360.0
    vertical: 30.0
  resolution:
    channels: 16
    angular: 0.2
  scan_rate: 10.0
  points_per_channel: 1800  # per revolution at 10Hz
  noise:
    enabled: true
    linear_stddev: 0.02
    angular_stddev: 0.001
```

## Camera Sensor Parameters

### Key Parameters
- **Resolution**: Width and height in pixels (e.g., 640x480, 1280x720)
- **Field of View (FOV)**: Usually expressed as horizontal FOV in radians or degrees
- **Frame Rate**: Images per second (e.g., 30, 60 FPS)
- **Image Format**: RGB8, BGR8, mono8, etc.
- **Lens Model**: Pinhole, distortion coefficients for fisheye
- **Noise**: Gaussian noise characteristics

### Example Configuration (RGB Camera)
```yaml
camera:
  resolution: [640, 480]
  fov_horizontal: 1.047  # ~60 degrees in radians
  frame_rate: 30.0
  format: "RGB8"
  noise:
    type: "gaussian"
    mean: 0.0
    std: 0.01
  distortion:
    k1: 0.0
    k2: 0.0
    p1: 0.0
    p2: 0.0
```

### Depth Camera Parameters
- **Depth Format**: 32FC1, 16UC1
- **Depth Range**: Min and max distances for depth measurement
- **Depth Accuracy**: How accurately depth is measured

## IMU Sensor Parameters

### Key Parameters
- **Measurement Rate**: How frequently measurements are made (e.g., 100-1000 Hz)
- **Noise Characteristics**:
  - Linear acceleration noise: Typical 0.01-0.1 m/s² RMS
  - Angular velocity noise: Typical 0.001-0.01 rad/s RMS
  - Orientation noise: When orientation is provided directly
- **Bias and Drift**: Systematic errors that develop over time
- **Dynamic Range**: Maximum measurable values

### Example Configuration (IMU)
```yaml
imu:
  update_rate: 100.0
  linear_acceleration:
    noise:
      type: "gaussian"
      mean: 0.0
      stddev: 0.017  # m/s²
  angular_velocity:
    noise:
      type: "gaussian"
      mean: 0.0
      stddev: 0.0015  # rad/s
  orientation:
    noise:
      type: "gaussian"
      mean: 0.0
      stddev: 0.01  # rad
  bias:
    linear_acceleration:
      x: 0.01
      y: 0.01
      z: 0.01
    angular_velocity:
      x: 0.001
      y: 0.001
      z: 0.001
```

## Force/Torque Sensor Parameters

### Key Parameters
- **Measurement Range**: Maximum forces/torques measurable (e.g., ±500N, ±50Nm)
- **Resolution**: Smallest detectable change in measurement
- **Update Rate**: How frequently measurements are updated
- **Noise Characteristics**: Gaussian noise models for each axis
- **Cross-talk**: How measurement on one axis affects others

### Example Configuration (Force/Torque)
```yaml
force_torque:
  update_rate: 1000.0
  force_range: [-500.0, 500.0]  # N for each axis
  torque_range: [-50.0, 50.0]   # Nm for each axis
  noise:
    force_stddev: 0.1  # N
    torque_stddev: 0.01  # Nm
  cross_talk: 0.01  # percentage of cross-axis influence
```

## Joint Position Sensor Parameters

### Key Parameters
- **Resolution**: Precision of angle measurement (e.g., 12-bit, 16-bit encoders)
- **Accuracy**: How close measurement is to true value
- **Update Rate**: How frequently position is measured
- **Range**: Valid measurement range (e.g., for revolute joints)

### Example Configuration (Joint Position)
```yaml
joint_position:
  resolution: 16  # bits
  accuracy: 0.001  # rad (about 0.057 degrees)
  update_rate: 1000.0
  noise:
    type: "gaussian"
    stddev: 0.0005  # rad
```

## Sensor Mounting and Positioning

### Coordinate System Considerations
- **Frame Definition**: How sensor frame is defined relative to robot body
- **Positioning**: Where sensor is physically located on the robot
- **Orientation**: How sensor is oriented relative to robot body

### Typical Mounting Positions
- **Cameras**: On head or chest for forward vision
- **LiDAR**: On head or torso for 360° view
- **IMU**: In torso/center of mass for balance
- **Force/Torque**: At joints or end-effectors for contact sensing

## Platform-Specific Configuration Examples

### Isaac Sim Configuration
```yaml
# Isaac Sim typically uses USD Prims for sensor definition
# Example for camera in USD format
def Camera "/World/Camera" (
    prepend apiSchemas = ["CameraSensorAPI"]
)
{
    float3 sensor:offset:position = (0, 0, 0)
    float3 sensor:offset:orientation = (0, 0, 0)
    float horizontal_fov = 1.047  # radians
    float vertical_fov = 0.785   # radians from aspect ratio
    int2 resolution = (640, 480)
    float min_range = 0.1
    float max_range = 10.0
}
```

### Gazebo Configuration (SDF)
```xml
<!-- Gazebo sensor definition in SDF -->
<sensor type="camera" name="head_camera">
  <update_rate>30.0</update_rate>
  <camera name="head_camera">
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.02</near>
      <far>300</far>
    </clip>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>camera</namespace>
      <remapping>~/image_raw:=/camera/image_raw</remapping>
    </ros>
  </plugin>
</sensor>
```

## Sensor Calibration Parameters

### Intrinsic Calibration
- **Focal Length**: In pixels for x and y axes
- **Principal Point**: Optical center in pixels
- **Distortion Coefficients**: For correcting lens distortion

### Extrinsic Calibration
- **Position**: 3D position relative to reference frame
- **Orientation**: Rotation relative to reference frame
- **Time Offset**: Delay between sensor measurements

## Performance Considerations

### Computational Impact
- **Update Rates**: Higher rates provide more data but require more computation
- **Resolution**: Higher resolution sensors create more data to process
- **Noise Models**: Complex noise models can impact simulation performance

### Optimization Strategies
- **Selective Simulation**: Only enable sensors when needed
- **Downsampling**: Reduce update rates where possible
- **Simplified Models**: Use less complex noise models when performance is critical

## Standard ROS Message Types and Parameters

### Common Message Types
- `sensor_msgs/LaserScan`: For 2D LiDAR
- `sensor_msgs/PointCloud2`: For 3D LiDAR
- `sensor_msgs/Image`: For camera images
- `sensor_msgs/Imu`: For IMU data
- `geometry_msgs/WrenchStamped`: For force/torque data

### Parameter Consistency
- Standardize parameters across different sensor types
- Use consistent units (SI units preferred)
- Follow ROS conventions for topic names and message structures

## References

1. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.
2. Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics. MIT Press.
3. Open Source Robotics Foundation. (2023). Gazebo Sensor Documentation.
4. NVIDIA. (2024). Isaac Sim Sensor Simulation Guide. NVIDIA Developer Documentation.
5. IEEE. (2015). IEEE Standard for Sensor Performance Specification. IEEE Std 2020.
6. Corke, P. (2017). Robotics, Vision and Control: Fundamental Algorithms in MATLAB. Springer.
7. Murphy, R. R. (2019). Introduction to AI Robotics. MIT Press.