---
title: Navigation Systems and Path Planning
sidebar_position: 1
---

# Navigation Systems and Path Planning

## Learning Objectives

After completing this chapter, you should be able to:

1. Understand the architecture of the Nav2 navigation stack
2. Configure navigation parameters for humanoid robot applications
3. Implement path planning algorithms for complex environments
4. Integrate perception systems with navigation planning
5. Evaluate navigation system performance in simulation environments

## Content

### Introduction to Navigation Systems

Navigation is a fundamental capability of autonomous robots, enabling them to move from one location to another in their environment safely and efficiently. For humanoid robots, navigation systems must account for complex kinematic constraints, balance requirements, and interaction with humans and objects in their environment.

The Navigation2 (Nav2) stack is the current standard for mobile robot navigation in ROS 2. It provides a comprehensive framework for path planning, obstacle avoidance, and motion control, with particular emphasis on modularity and flexibility.

Key components of navigation systems include:

- **Localization**: Determining the robot's position in the environment
- **Path Planning**: Computing a valid route from start to goal
- **Motion Control**: Executing the planned path while avoiding obstacles
- **Mapping**: Creating and updating environmental representations

### Architecture of the Nav2 Stack

The Nav2 stack is built around a service-oriented architecture with clearly defined interfaces between components. The main components work together to provide complete navigation functionality.

#### Navigation Server
The Navigation Server acts as the main coordinator for navigation tasks:

```python
# Example Navigation Server configuration
from nav2_behavior_tree.bt_runner import BehaviorTreeRunner
from nav2_msgs.action import NavigateToPose

class NavigationServer:
    def __init__(self):
        # Initialize action server
        self.navigation_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.navigate_to_pose_callback,
            callback_group=self.callback_group,
            result_timeout=600  # 10 minutes timeout
        )
    
    def navigate_to_pose_callback(self, goal_handle):
        # Process navigation goal
        # Coordinate with global/local planners and controllers
        pass
```

#### Costmap Architecture
Costmaps provide spatial representations of the environment for planning and obstacle avoidance:

```yaml
# Example costmap configuration
costmap:
  ros__parameters:
    # Costmap resolution and dimensions
    resolution: 0.05  # meters per cell
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    
    # Static map layer
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True
      
    # Obstacle detection layer  
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
        
    # Inflation layer for safety margins
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.8
```

### Path Planning Algorithms

Path planning algorithms compute feasible routes from the robot's current position to a goal location. Different algorithms are suitable for different scenarios.

#### Global Path Planning

Global planners compute the overall path from start to goal considering static obstacles:

```python
from nav2_msgs.action import ComputePathToPose

class GlobalPlanner:
    def __init__(self):
        self.planner_types = {
            'navfn': 'nav2_navfn_planner/NavfnPlanner',
            'grid_based': 'nav2_navfn_planner/NavfnPlanner',
            'carrot_planner': 'nav2_carrot_planner/CarrotPlanner',
            'smac_planner': 'nav2_smac_planner/SMACPlanner'
        }
        
    def compute_path(self, start_pose, goal_pose, planner_type='navfn'):
        # Compute path using selected planner
        if planner_type == 'navfn':
            return self.navfn_plan(start_pose, goal_pose)
        elif planner_type == 'smac':
            return self.smac_plan(start_pose, goal_pose)
        else:
            raise ValueError(f"Unknown planner type: {planner_type}")
    
    def navfn_plan(self, start_pose, goal_pose):
        # Implementation of NavFn path planning
        # Convert start and goal to costmap coordinates
        # Run Dijkstra's algorithm or A*
        pass
        
    def smac_plan(self, start_pose, goal_pose):
        # Implementation of SMAC (Sparse-Markov Chain) planning
        # Use SE2 or SO(2) planning with holonomic robots
        pass
```

##### Common Global Planners

**NavFn (Dijkstra-based Planner)**:
- Uses Dijkstra's algorithm to compute optimal path
- Good for general purpose navigation
- Can handle unknown space with appropriate parameters

**A* Planner**:
- Variant of Dijkstra's algorithm with heuristic for faster planning
- Uses Manhattan or Euclidean distance heuristic
- More efficient than NavFn for large maps

**Carrot Planner**:
- Finds a valid pose near the goal if exact goal is blocked
- Useful for navigation to occupied spaces
- Good for delivery or approach scenarios

**SMAC Planner**:
- Sparse-Markov Chain planner for SE2 or SO(2) spaces
- More efficient than grid-based methods for certain robot types
- Better integration with local planners for smooth navigation

#### Local Path Planning

Local planners execute the global path while avoiding dynamic obstacles:

```python
from nav2_msgs.action import FollowPath

class LocalPlanner:
    def __init__(self):
        self.controller_types = {
            'pure_pursuit': 'nav2_regulated_pure_pursuit_controller/RegulatedPurePursuitController',
            'dwb': 'dwb_core::DWBLocalPlanner',
            'teb': 'nav2_teb_local_planner::TEBLocalPlanner'
        }
        
    def follow_path(self, path, controller_type='pure_pursuit'):
        if controller_type == 'pure_pursuit':
            return self.pure_pursuit_follow(path)
        elif controller_type == 'dwb':
            return self.dwb_follow(path)
        elif controller_type == 'teb':
            return self.teb_follow(path)
        else:
            raise ValueError(f"Unknown controller type: {controller_type}")
    
    def pure_pursuit_follow(self, path):
        # Implementation of regulated pure pursuit
        # Calculate lookahead point
        # Compute linear and angular velocities
        pass
```

##### Common Local Controllers

**Regulated Pure Pursuit**:
- Pure pursuit with collision regulation
- Maintains speed based on obstacle proximity
- Good for differential drives

**DWB (Dynamic Window Approach)**:
- Considers robot's kinodynamic constraints
- Evaluates multiple trajectories based on robot capabilities
- Good for dynamic obstacle avoidance

**TEB (Timed Elastic Band)**:
- Real-time trajectory optimization
- Considers time dimension for dynamic avoidance
- Good for human-aware navigation

### Navigation Parameters for Humanoid Robots

Humanoid robots have specific requirements that must be considered in navigation configuration:

#### Physical Constraints

**Footprint Configuration**:
```yaml
# Humanoid robot footprint configuration
robot_description: "humanoid_robot"
footprint: [ [-0.7, -0.4], [-0.7, 0.4], [0.7, 0.4], [0.7, -0.4] ]
# Or for circular approximation:
# robot_radius: 0.6
```

**Kinematic Constraints**:
```yaml
# Conservative velocities for humanoid stability
max_vel_x: 0.4
min_vel_x: 0.1
max_vel_theta: 0.5
min_vel_theta: -0.5
acc_lim_x: 0.2
acc_lim_theta: 0.3
decel_lim_x: 0.4  # Can brake faster than accelerate
```

#### Safety Parameters

**Increased Safety Margins**:
```yaml
# More conservative inflation for humanoid safety
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  cost_scaling_factor: 5.0  # More aggressive inflation
  inflation_radius: 1.0  # 1m safety buffer
```

**Balanced Performance vs. Safety**:
```yaml
# Conservative navigation parameters for stability
local_planner:
  ros__parameters:
    desired_linear_vel: 0.3  # Slower than default for stability
    max_linear_accel: 0.2   # Lower acceleration for balance
    max_angular_accel: 0.4  # Controlled turning
    min_approach_dist: 0.5  # Maintain distance from obstacles during approach
```

### Behavior Trees for Navigation

Behavior trees provide a flexible way to define navigation execution sequences and recovery behaviors:

#### Navigation Behavior Tree Structure
```xml
<!-- Example behavior tree for humanoid navigation -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPoseRecovery">
            <Sequence name="ComputePathToPoseRequest">
              <GoalChecker id="goal_checker"/>
              <ComputePathToPose id="Path"/>
            </Sequence>
            <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
              <GoalReached id="GoalChecker"/>
              <ComputePathToPose id="ComputePathController"/>
            </ReactiveFallback>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="2" name="FollowPathRecovery">
          <FollowPath id="FollowPath"/>
          <ReactiveFallback name="FollowPathRecoveryFallback">
            <GoalReached id="GoalChecker"/>
            <FollowPath id="FollowPathController"/>
          </ReactiveFallback>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="NavigateRecoveryFallback">
        <GlobalDesigner id="MoveRobotDesigner"/>
        <RetryNode number_of_retries="4">
          <RecoveryNode number_of_retries="2" name="spin">
            <Spin spin_dist="1.57"/>  <!-- Turn 90 degrees -->
          </RecoveryNode>
        </RetryNode>
        <RetryNode number_of_retries="4">
          <RecoveryNode number_of_retries="2" name="backup">
            <BackUp backup_dist="0.30" backup_speed="0.05"/>
          </RecoveryNode>
        </RetryNode>
        <RecoveryNode number_of_retries="2" name="wait">
          <Wait wait_duration="5"/>
        </RecoveryNode>
        <RecoveryNode number_of_retries="2" name="balance_recovery">
          <BalanceRecovery />  <!-- Custom humanoid behavior -->
        </RecoveryNode>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

#### Custom Humanoid Behaviors
For humanoid robots, you might need to implement custom behaviors:

```python
import py_trees
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class BalanceRecovery(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name)
        
    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.cmd_vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.balance_pub = self.node.create_publisher(Bool, '/balance_control', 10)
        
    def initialise(self):
        pass  # Prepare for balance recovery
        
    def update(self):
        # Send balance recovery command
        balance_msg = Bool()
        balance_msg.data = True
        self.balance_pub.publish(balance_msg)
        
        # Stop all motion during balance recovery
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Assume balance recovery takes 2 seconds
        # In real implementation, this would check balance sensors
        return py_trees.common.Status.SUCCESS
```

### Path Planning for Complex Environments

Humanoid robots often need to navigate in complex, dynamic environments with humans, furniture, and other obstacles.

#### Dynamic Obstacle Avoidance
```python
# Example of handling dynamic obstacles
class DynamicAvoidance:
    def __init__(self):
        self.tracking_manager = ObjectTracker()
        self.prediction_engine = Motion Predictor()
        
    def avoid_dynamic_obstacles(self, robot_pose, path):
        # Track dynamic obstacles
        tracked_objects = self.tracking_manager.get_tracked_objects()
        
        # Predict future positions
        predicted_positions = self.prediction_engine.predict(tracked_objects, 2.0)  # 2 sec prediction
        
        # Modify path based on predicted movements
        safe_path = self.replan_with_predictions(path, predicted_positions)
        
        return safe_path
```

#### Human-Aware Navigation
For navigation around humans:

```python
# Human-aware navigation parameters
social_navigation:
  ros__parameters:
    # Respect personal space
    personal_space_radius: 1.0  # 1m personal space
    social_temporal_reasoning: True  # Consider temporal aspects
    
    # Proxemic zones (Hall, 1966)
    intimate_zone: 0.45  # Very close interaction
    personal_zone: 1.2   # Regular social interaction  
    social_zone: 3.7     # Social distance
    public_zone: 8.0     # Public distance
    
    # Adjust behavior based on zones
    approach_strategies:
      - zone: "personal"
        strategy: "slow_approach"
        min_dist: 1.0
      - zone: "social" 
        strategy: "normal"
        min_dist: 2.0
```

### Performance Evaluation and Optimization

Evaluating navigation performance is critical for humanoid robots:

#### Performance Metrics
- **Success Rate**: Percentage of successful navigation attempts
- **Path Efficiency**: Ratio of path length to straight-line distance
- **Execution Time**: Time taken to reach the goal
- **Safety Incidents**: Number of emergency stops or near-collisions
- **Deviation from Path**: How closely the robot follows the planned path

#### Optimization Strategies
- **Parameter Tuning**: Systematic adjustment of navigation parameters
- **Algorithm Selection**: Choosing appropriate algorithms for the task
- **Sensor Fusion**: Integrating multiple sensor types for better navigation
- **Adaptive Behavior**: Adjusting navigation behavior based on context

## Lab Exercise

### Setup

Before starting this lab, ensure you have:

- Nav2 installed and configured (ROS 2 Humble or newer)
- Isaac Sim or Gazebo for testing navigation
- A map of the environment (or use the turtlebot3 simulation world)
- Understanding of basic ROS 2 concepts

### Procedure

#### Step 1: Configure the Navigation Stack
- **Commands:**
  ```
  # Create a navigation configuration file
  # Edit nav2_params.yaml with basic humanoid navigation parameters
  
  # Set up the footprint for a humanoid robot
  footprint: [ [-0.7, -0.4], [-0.7, 0.4], [0.7, 0.4], [0.7, -0.4] ]
  
  # Configure conservative velocities for humanoid stability
  max_vel_x: 0.4
  max_vel_theta: 0.5
  acc_lim_x: 0.2
  acc_lim_theta: 0.3
  ```
- **Expected Result:** Navigation parameters file with humanoid-appropriate settings

#### Step 2: Launch Navigation in Simulation
- **Commands:**
  ```
  # Launch Gazebo simulation with a robot
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
  
  # In a separate terminal, launch navigation
  ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=True \
    params_file:=/path/to/your/nav2_params.yaml
  ```
- **Expected Result:** Navigation stack launches without errors

#### Step 3: Test Path Planning
- **Commands:**
  ```
  # Launch RViz for visualization
  ros2 launch nav2_bringup rviz_launch.py
  
  # In RViz:
  1. Set initial pose for the robot
  2. Send a navigation goal using the "Nav2 Goal" tool
  3. Observe the path planning and execution
  ```
- **Expected Result:** Robot plans and follows a path to the goal position

#### Step 4: Test Obstacle Avoidance
- **Commands:**
  ```
  # Add obstacles in Gazebo or use the existing environment
  # Send navigation goals near obstacles
  # Observe the robot's obstacle avoidance behavior
  # Try different speeds and accelerations
  ```
- **Expected Result:** Robot avoids obstacles while maintaining stability

#### Step 5: Evaluate Navigation Performance
- **Commands:**
  ```
  # Record navigation metrics
  # Test different parameters to adjust behavior
  # Document the optimal settings for your environment
  ```
- **Expected Result:** Understanding of how different parameters affect navigation performance

## Expected Output

After completing this lab, you should have:

1. Configured a navigation stack with humanoid-specific parameters
2. Successfully run the navigation stack in simulation
3. Observed path planning and obstacle avoidance behavior
4. Adjusted navigation parameters based on performance
5. Understood the relationship between parameters and navigation behavior

## Troubleshooting Tips

- **Path planning fails**: Check map quality, inflation parameters, and goal validity
- **Robot stops unexpectedly**: Check sensor data, safety margins, and obstacle detection
- **Unstable behavior**: Reduce velocities and accelerations, increase safety margins
- **Oscillation**: Adjust local planner parameters, especially lookahead distances

## Comprehension Check

1. What is the main purpose of the Nav2 navigation stack in robotics?
   - A) To control robot vision systems
   - B) To enable mobile robots to navigate from one location to another safely and efficiently
   - C) To process sensor data only
   - D) To plan paths only, without execution
   
   **Correct Answer:** B
   **Explanation:** The Nav2 navigation stack enables mobile robots to navigate from one location to another safely and efficiently, including path planning, obstacle avoidance, and motion control.

2. What is the primary difference between global and local planners?
   - A) Global planners are faster than local planners
   - B) Global planners focus on short-term path execution, local planners on overall route planning
   - C) Global planners plan overall route considering static obstacles, local planners execute path while avoiding dynamic obstacles
   - D) Local planners are only for indoor environments
   
   **Correct Answer:** C
   **Explanation:** Global planners compute the overall route from start to goal considering static obstacles, while local planners execute the path while avoiding dynamic obstacles in real-time.

3. Which factor is particularly important for humanoid navigation compared to wheeled robots?
   - A) Speed of navigation
   - B) Balance and stability requirements
   - C) Camera resolution
   - D) Color of the robot
   
   **Correct Answer:** B
   **Explanation:** Balance and stability requirements are particularly critical for humanoid robots due to their bipedal nature, unlike wheeled robots which have inherent stability.

4. What does the inflation layer in costmaps do?
   - A) Increases robot speed
   - B) Makes maps more colorful
   - C) Adds safety margins around obstacles based on robot size
   - D) Removes obstacles from the map
   
   **Correct Answer:** C
   **Explanation:** The inflation layer adds safety margins around obstacles based on the robot's size and the desired safety distance, preventing collisions.

5. What is the main advantage of behavior trees in navigation?
   - A) They reduce the need for sensors
   - B) They provide flexible definition of navigation sequences and recovery behaviors
   - C) They eliminate the need for path planning
   - D) They make robots faster
   
   **Correct Answer:** B
   **Explanation:** Behavior trees provide a flexible way to define navigation execution sequences and recovery behaviors, allowing for complex, adaptive navigation strategies.

6. Why are conservative velocities important for humanoid robots?
   - A) To save battery power
   - B) To maintain balance and stability during navigation
   - C) To reduce sensor requirements
   - D) To improve communication speed
   
   **Correct Answer:** B
   **Explanation:** Conservative velocities are important for humanoid robots to maintain balance and stability during navigation, as they have complex balance requirements unlike wheeled robots.

7. What is the purpose of the local costmap in navigation?
   - A) To store the entire map of the world
   - B) To provide a rolling window of nearby obstacles for motion control
   - C) To save navigation parameters only
   - D) To store camera images only
   
   **Correct Answer:** B
   **Explanation:** The local costmap provides a rolling window of nearby obstacles and environmental features for motion control and local path adjustment.

8. Which algorithm is NOT a global path planning method?
   - A) A*
   - B) Dijkstra's algorithm
   - C) Dynamic Window Approach
   - D) SMAC Planner
   
   **Correct Answer:** C
   **Explanation:** Dynamic Window Approach is a local path planning method, not a global planning method. The others (A*, Dijkstra's, SMAC) are global planning algorithms.

## Summary

This chapter covered the critical aspects of navigation systems and path planning for humanoid robots, including the architecture of the Nav2 stack, configuration of parameters, and planning algorithms. Humanoid robots have specific requirements for navigation due to their balance and kinematic constraints, which require careful consideration in configuration and parameter selection. Understanding these concepts is essential for developing humanoid robots capable of autonomous navigation in complex environments.

## References

1. Macenski, S., et al. (2022). Navigation2: A Navigation Framework for Ground Mobile Robots in ROS 2. Journal of Open Source Software.
2. Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance. IEEE Robotics & Automation Magazine.
3. Khatib, O., et al. (2018). Humanoid Robotics: A Reference. MIT Press.
4. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.
5. Rosmann, C., et al. (2017). Trajectory modification considering dynamic constraints. International Conference on Control, Automation, Robotics and Vision.
6. Howard, T., & Kelly, A. (2007). Optimal Rough Terrain Trajectory Generation for Wheeled Mobile Robots. International Journal of Robotics Research.
7. Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to Collision Avoidance. IEEE Robotics & Automation Magazine. This foundational paper introduces the dynamic window approach for collision avoidance, a key concept in mobile robot navigation that remains relevant for humanoid navigation systems.

8. Khatib, O., et al. (2018). Humanoid Robotics: A Reference. MIT Press. This comprehensive reference provides essential background on humanoid robotics including navigation and locomotion principles specific to bipedal robots.

9. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer. A comprehensive handbook that covers all aspects of robotics including navigation systems for various robot types including humanoids.

10. Howard, T., & Kelly, A. (2007). Optimal Rough Terrain Trajectory Generation for Wheeled Mobile Robots. International Journal of Robotics Research. This paper provides insights into trajectory generation that applies to complex terrain navigation relevant to humanoid robots.

11. Rosmann, C., et al. (2017). Trajectory modification considering dynamic constraints. International Conference on Control, Automation, Robotics and Vision. This research addresses trajectory optimization with dynamic constraints, which is particularly relevant for the complex kinematics of humanoid robots.