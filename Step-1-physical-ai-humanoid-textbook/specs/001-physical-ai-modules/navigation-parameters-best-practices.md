# Navigation Parameters and Best Practices for Humanoid Robotics

## Overview
This document provides a comprehensive guide to configuring navigation parameters specifically for humanoid robots, along with best practices for creating stable and efficient navigation systems. The parameters are tailored to the unique requirements of humanoid robots, including balance considerations and complex kinematic constraints.

## Core Navigation Parameters for Humanoid Robots

### 1. Robot Characterization Parameters

#### Physical Dimensions
- **Footprint**: Accurate robot footprint is crucial for safe navigation
  ```yaml
  # For humanoid robots, a larger, more rectangular footprint may be needed
  footprint: [[-0.7, -0.4], [-0.7, 0.4], [0.7, 0.4], [0.7, -0.4]]
  # Or circular approximation
  robot_radius: 0.6
  ```
- **Dimensions**: Used for collision checking and path planning
  - Length: ~1.4m (full leg span)
  - Width: ~0.8m (shoulder width)
  - Height: ~1.5-2.0m (depending on robot)

#### Kinematic Constraints
- **Linear Velocities**:
  - Max: 0.5 m/s (conservative for humanoid stability)
  - Min: 0.1 m/s (ensure smooth motion)
  - Acceleration: 0.2 m/s² (gentle acceleration for balance)
  
- **Angular Velocities**:
  - Max: 0.6 rad/s (limited by turning mechanics)
  - Acceleration: 0.3 rad/s² (conservative for balance)

#### Differential Drive Parameters (for wheeled base if applicable)
```yaml
# Kinematic constraints for humanoid with wheeled base
max_vel_x: 0.5
min_vel_x: -0.2
max_vel_y: 0.0  # No lateral movement for differential drive
max_vel_theta: 0.6
min_vel_theta: -0.6
min_turn_radius: 0.4  # Minimum turning radius
acc_lim_x: 0.2
acc_lim_theta: 0.3
```

### 2. Costmap Configuration Parameters

#### Global Costmap Parameters
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      # Update frequencies
      update_frequency: 1.0  # Hz - sufficient for path planning
      publish_frequency: 1.0  # Hz - less frequent for global map
      
      # Robot dimensions and safety
      robot_base_frame: base_link
      footprint: [ [-0.7, -0.4], [-0.7, 0.4], [0.7, 0.4], [0.7, -0.4] ]
      resolution: 0.05  # 5cm resolution for humanoid precision
      
      # Origin and size - use rolling window for dynamic environments
      origin_x: 0.0
      origin_y: 0.0
      width: 100  # 5m x 5m area (resolution * width = 5m)
      height: 100
      
      # Plugin configuration
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          obstacle_range: 4.0  # 4m obstacle detection
          raytrace_range: 5.0  # 5m raytracing
          clearing: True
          marking: True
          data_type: "LaserScan"
          
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0  # Aggressive inflation for humanoid safety
        inflation_radius: 1.0  # 1m inflation radius
        inflate_unknown: False  # Don't inflate unknown areas
```

#### Local Costmap Parameters
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Update frequencies - higher for local map due to safety
      update_frequency: 5.0  # Hz - more frequent updates for safety
      publish_frequency: 2.0  # Hz - more frequent publishing
      
      # Robot dimensions and safety
      robot_base_frame: base_link
      footprint: [ [-0.7, -0.4], [-0.7, 0.4], [0.7, 0.4], [0.7, -0.4] ]
      resolution: 0.025  # Higher resolution for local planning
      
      # Rolling window for dynamic navigation
      rolling_window: True
      width: 8  # 4m x 4m local area
      height: 8
      origin_x: -4.0  # Center the window around the robot
      origin_y: -4.0
      
      # Plugin configuration
      plugins: ["voxel_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 5.0
          raytrace_min_range: 0.0
          obstacle_max_range: 4.0
          obstacle_min_range: 0.1
          
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2  # 20cm voxel height
        z_voxels: 10  # 2m tall voxels
        max_obstacle_height: 2.0
        mark_threshold: 0  # Mark if one obstacle found
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 5.0  # More aggressive inflation for local safety
        inflation_radius: 0.8  # 80cm inflation radius for local map
```

### 3. Planner Parameters

#### Global Planner Configuration
```yaml
global_planner:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5  # Acceptable distance to goal
      use_astar: False  # A* can be more computationally expensive
      allow_unknown: True  # Allow planning through unknown space
```

#### Local Planner Configuration (Regulated Pure Pursuit)
```yaml
local_planner:
  ros__parameters:
    controller_frequency: 20.0  # Higher frequency for humanoid stability
    controller_plugin_ids: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController"
      desired_linear_vel: 0.3  # Conservative linear velocity for stability
      max_linear_accel: 0.3  # Conservative acceleration
      max_linear_decel: 0.5  # Can decel faster than accel
      lookahead_dist: 0.6  # Lookahead distance for smooth navigation
      min_lookahead_dist: 0.3  # Minimum lookahead
      max_lookahead_dist: 0.9  # Maximum lookahead
      lookahead_time: 1.5  # Time-based lookahead
      rotate_to_heading_angular_dist: 0.785  # Rotate to heading at 45 degrees
      rotate_to_heading_rate: 1.0  # Rate limit for rotation
      max_angular_accel: 0.6  # Conservative angular acceleration
      goal_dist_tol: 0.25  # Tolerance for reaching goal
      xy_goal_tolerance: 0.25  # XY tolerance for goal
      trans_stopped_velocity: 0.25  # Velocity to consider stopped
      short_circuit_trajectory: True  # Optimize trajectory execution
```

### 4. Behavior Tree Parameters
```yaml
bt_navigator:
  ros__parameters:
    # Behavior tree XML file
    bt_loop_duration: 10  # Maximum time for BT execution
    info_timeout: 500
    node_timeout: 2000
    action_server_result_timeout: 900  # 15 minutes for navigation
    enable_groot_monitoring: False  # Disable unless debugging
    enable_action_monitoring: True  # Monitor action execution
    interrupt_handle_bt_duration: 100  # ms for interruption handling
    
    # Recovery behaviors
    retry_patience: 1.0  # Time to wait before recovery
    tree_error_tolerance: 1  # Number of failures before giving up
```

## Best Practices for Humanoid Navigation

### 1. Safety First Approach

#### Conservative Parameter Setting
- **Lower velocities**: Set conservative velocity limits to ensure stability
- **Higher safety margins**: Increase inflation radii to avoid potential obstacles
- **Conservative accelerations**: Gentle acceleration for humanoid balance

#### Failure Handling
- **Graceful degradation**: Ensure robot can safely stop when navigation fails
- **Recovery behaviors**: Implement appropriate recovery behaviors for humanoid robots
- **Human intervention**: Allow for human override when needed

### 2. Stability and Balance Considerations

#### Center of Mass Management
- **Slow speed navigation**: Reduce speeds to maintain balance during movement
- **Smooth trajectories**: Avoid sudden direction changes
- **Predictable motion**: Use motion profiles that maintain stability

```yaml
# Example motion profile configuration for stability
motion_control:
  ros__parameters:
    # Trajectory smoothing parameters
    trajectory_smoothing_enabled: True
    smoothing_lambda: 0.01  # Low smoothing parameter for smoother motion
    max_trajectory_deviation: 0.3  # Max deviation from planned path
    
    # Balance-focused parameters
    balance_margin: 0.2  # Additional margin for balance
    dynamic_stability: True  # Enable dynamic stability checks
    com_tracking_enabled: False  # Enable center of mass tracking if available
```

### 3. Environmental Adaptation

#### Terrain Awareness
- **Ground plane detection**: Use depth sensors for ground plane detection
- **Step height limitation**: Configure maximum step height for humanoid
- **Slope detection**: Implement slope detection and avoidance

#### Dynamic Obstacle Handling
```yaml
# Dynamic obstacle handling parameters
dynamic_obstacles:
  ros__parameters:
    # Detection parameters
    detection_range: 2.0  # Range for detecting dynamic obstacles
    detection_frequency: 10.0  # Frequency for dynamic obstacle detection
    
    # Handling parameters
    dynamic_inflation_factor: 2.0  # Inflation multiplier for moving obstacles
    velocity_check_threshold: 0.5  # Velocity threshold for dynamic detection
    prediction_horizon: 3.0  # Time horizon for predicting obstacle movement
```

### 4. Sensor Integration

#### Multi-Sensor Fusion
- **LiDAR for navigation**: Primary sensor for obstacle detection
- **Vision for semantics**: Use cameras for semantic understanding
- **IMU for stability**: IMU for motion and stability control

#### Sensor-Specific Configuration
```yaml
# LiDAR-specific configuration
lidar_configuration:
  ros__parameters:
    # Obstacle detection
    obstacle_range: 3.0  # Range for obstacle detection
    voxel_size: 0.1  # Voxel size for obstacle clustering
    
    # Ground plane filtering
    ground_filter_enabled: True
    ground_normal_threshold: 0.1  # Threshold for ground detection
    ground_negative_threshold: -0.2  # Threshold for ground rejection
    
# Camera integration (if used for semantic navigation)
camera_integration:
  ros__parameters:
    # Semantic map integration
    semantic_layer_enabled: True
    semantic_topic: "/semantic_map"
    object_detection_timeout: 5.0  # Timeout for object detection
```

### 5. Performance Optimization

#### Computational Efficiency
- **Appropriate update rates**: Balance accuracy with computational load
- **Efficient algorithms**: Choose algorithms that work well with humanoid constraints
- **Multi-threading**: Use multi-threading where possible

#### Memory Management
- **Map resolution**: Balance map detail with memory requirements
- **Costmap size**: Configure appropriate costmap sizes for your environment
- **Data buffering**: Use appropriate buffer sizes for sensor data

### 6. Testing and Validation Protocols

#### Simulation Testing
- **Isaac Sim**: Test navigation in Isaac Sim with realistic physics
- **Gazebo**: Use Gazebo for basic navigation functionality tests
- **Scenario validation**: Test on multiple scenarios before real-world deployment

#### Real-World Testing
- **Progressive complexity**: Start with simple environments and increase complexity
- **Safety protocols**: Always have emergency stop procedures
- **Data collection**: Collect navigation performance data for parameter adjustment

### 7. Parameter Tuning Strategies

#### Systematic Approach
1. **Initial parameters**: Start with conservative parameters
2. **Incremental adjustments**: Make small adjustments to one parameter at a time
3. **Validation**: Test thoroughly after each adjustment
4. **Documentation**: Keep records of parameter changes and results

#### Performance Metrics
- **Success rate**: Percentage of successful navigation attempts
- **Path efficiency**: Ratio of path length to straight-line distance
- **Execution time**: Time to reach the goal
- **Safety incidents**: Number of safety-related stops or adjustments

### 8. Specific Humanoid Considerations

#### Bipedal Walking Integration
```yaml
# Bipedal-specific navigation parameters
bipedal_integration:
  ros__parameters:
    # Walking pattern considerations
    gait_adaptation_enabled: True  # Enable gait adaptation during navigation
    step_timing_consideration: True  # Consider step timing in path planning
    balance_check_interval: 1.0  # Balance check frequency during navigation
    
    # Footstep planning interface
    footstep_planner_enabled: False  # Enable if using footstep planner
    step_height_limit: 0.1  # Maximum step height
    step_width_limit: 0.3  # Maximum step width
```

#### Human Interaction Parameters
- **Social navigation**: Implement social navigation parameters
- **Personal space**: Respecting human personal space (typically 0.8m+)
- **Approach behavior**: Proper approach behavior for HRI scenarios

```yaml
# Social navigation parameters
social_navigation:
  ros__parameters:
    personal_space_radius: 1.0  # Respect human personal space
    social_temporal_reasoning: True  # Consider temporal aspects of social navigation
    pedestrian_avoidance_enabled: True  # Enable pedestrian avoidance
    interaction_distance: 2.0  # Distance to trigger interaction behaviors
    
    # Proxemic zones
    intimate_zone: 0.45  # Very close interaction
    personal_zone: 1.2  # Regular social interaction
    social_zone: 3.7  # Social distance
    public_zone: 8.0  # Public distance
```

## Troubleshooting Guidelines

### Common Configuration Issues

#### 1. Oscillation During Navigation
- **Cause**: Local planner goals are near obstacles
- **Solution**: Increase min_lookahead_dist, decrease desired_linear_vel
- **Parameters to adjust**: `min_lookahead_dist`, `desired_linear_vel`

#### 2. Getting Stuck Near Obstacles
- **Cause**: Excessive inflation or tight navigation areas
- **Solution**: Adjust inflation_radius or increase robot footprint allowance
- **Parameters to adjust**: `inflation_radius`, `cost_scaling_factor`

#### 3. Erratic Motion
- **Cause**: High velocities or accelerations for humanoid stability
- **Solution**: Reduce linear/angular velocities and accelerations
- **Parameters to adjust**: `max_linear_vel`, `max_angular_vel`, `max_lin_acc`

### Performance Optimization

#### 1. High CPU Usage
- **Reduce update frequencies**: Lower costmap update rates
- **Decrease resolution**: Lower costmap resolution appropriately
- **Disable unused plugins**: Turn off unused costmap layers

#### 2. Memory Issues
- **Decrease costmap sizes**: Reduce local/global costmap dimensions
- **Reduce history**: Limit trajectory buffers
- **Optimize data types**: Use appropriate data types for sensors

## References and Standards

1. Macenski, S., et al. (2022). "Navigation2: A Navigation Framework for Ground Mobile Robots in ROS 2". Journal of Open Source Software.
2. Khatib, O., et al. (2018). "Humanoid Robotics: A Reference". MIT Press.
3. Siciliano, B., & Khatib, O. (2016). "Springer Handbook of Robotics". Springer.
4. Fox, D., et al. (1997). "The Dynamic Window Approach to Collision Avoidance". IEEE Robotics & Automation Magazine.
5. Open Source Robotics Foundation. (2023). "Navigation2 Configuration Guide". ROS Navigation Working Group.
6. Rosmann, C., et al. (2017). "Trajectory modification considering dynamic constraints". International Conference on Control, Automation, Robotics and Vision.
7. Howard, T., & Kelly, A. (2007). "Optimal Rough Terrain Trajectory Generation for Wheeled Mobile Robots". International Journal of Robotics Research.