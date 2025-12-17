# Gravity, Time Step, and Solver Settings for Physics Simulation

## Overview
This document details the critical physics parameters for humanoid robot simulation: gravity, time step (dt), and solver settings. These parameters are fundamental to achieving realistic and stable physics simulation in environments like Isaac Sim and Gazebo.

## Gravity Settings

### Standard Gravity
- **Value**: 9.80665 m/s² (often approximated as 9.81 m/s²)
- **Direction**: Typically (0, 0, -9.81) in a right-handed coordinate system
- **Significance**: 
  - Primary force affecting all objects in the simulation
  - Critical for realistic motion, balance, and contact interactions
  - Essential for humanoid locomotion and stability algorithms

### Gravity Configuration Examples

#### Isaac Sim
```python
# Setting gravity in Isaac Sim via Python API
from omni.isaac.core import World
world = World()
world.scene.set_physics_params(gravity=[0.0, 0.0, -9.81])
```

#### Gazebo
```xml
<!-- Setting gravity in Gazebo world file -->
<world name="default">
  <physics type="ode">
    <gravity>0 0 -9.8</gravity>
    <!-- other physics parameters -->
  </physics>
</world>
```

### Alternative Gravity Values
- **Moon**: ~1.62 m/s² (for lunar robotics simulation)
- **Mars**: ~3.71 m/s² (for Martian robotics simulation)
- **Jupiter**: ~24.79 m/s² (for theoretical high-gravity simulation)

## Time Step (dt) Settings

### Definition and Importance
- **Definition**: The time interval between successive physics simulation updates
- **Significance**: Affects both simulation accuracy and computational performance
- **Trade-offs**: Smaller steps provide more accurate results but require more computation

### Recommended Time Step Values

#### For Humanoid Simulation
- **Physics Update Rate**: 1000 Hz (0.001s time step) - Standard
- **Real-time Factor**: 1.0 for real-time simulation
- **Maximum Step Size**: 0.001s to 0.01s depending on dynamics

#### Relationship to Controller Frequency
- If humanoid controllers run at 100 Hz, physics should be at least 100 Hz
- For 1000 Hz control, physics simulation should match or exceed this rate
- Higher physics rates enable more responsive control but increase computation

### Time Step Configuration Examples

#### Isaac Sim
```yaml
# Isaac Sim configuration file
simulation:
  physics_dt: 0.001  # 1ms time step
  rendering_dt: 0.01667  # ~60 FPS for rendering
  stage_units_in_meters: 1.0
```

#### Gazebo
```xml
<!-- Gazebo world file -->
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- 1ms time step -->
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>  <!-- Updates per second -->
</physics>
```

## Solver Settings

### Solver Types

#### Sequential Impulse Solver
- **Use Case**: Game physics, robotics
- **Advantages**: Good performance, handles contacts well
- **Disadvantages**: Less accurate for complex constraints

#### Projected Gauss-Seidel (PGS)
- **Use Case**: General physics simulation
- **Advantages**: Good balance of accuracy and performance
- **Disadvantages**: Can be slow for complex systems

#### TGS (Two Gauss-Seidel)
- **Use Case**: Isaac Sim's preferred solver
- **Advantages**: Better stability and accuracy
- **Disadvantages**: Higher computational cost

### Solver Parameters

#### Iteration Counts
- **Position Iterations**: Controls position accuracy (typically 4-8 for humanoid)
- **Velocity Iterations**: Controls velocity accuracy (typically 1-4 for humanoid)
- **Higher values**: More accurate but slower simulation

#### Solver Configuration Examples

##### Isaac Sim
```yaml
physics:
  solver_type: "TGS"  # Two Gauss-Seidel solver
  num_position_iterations: 8  # Higher for more accuracy
  num_velocity_iterations: 2
  enable_stabilization: true
  stabilization_threshold: 0.001
```

##### Gazebo (ODE Solver)
```xml
<physics type="ode">
  <ode>
    <solver type="quick" iters="10" sor="1.0" />
    <!-- iters = iteration count -->
    <!-- sor = successive over-relaxation parameter -->
  </ode>
</physics>
```

### Constraints and Stabilization

#### Constraint Force Mixing (CFM)
- **Purpose**: Helps stabilize constraints
- **Typical Values**: 0.0 (default), small positive values for stability

#### Error Reduction Parameter (ERP)
- **Purpose**: Controls how quickly constraint errors are corrected
- **Typical Values**: 0.1 to 0.8 (0.2 is common default)

#### Contact Parameters
- **Max Correcting Velocity**: Limit on contact correction speed
- **Surface Layer**: Small buffer to prevent interpenetration

## Practical Considerations for Humanoid Robots

### Balance and Stability
- Accurate gravity is essential for balance algorithms
- Proper time step ensures stable controller interaction
- Solver settings must support complex contact scenarios (e.g., feet on ground)

### Computational Performance
- Larger time steps reduce computation but may cause instability
- Higher solver iterations improve accuracy but increase computation
- Consider real-time constraints for interactive simulation

### Tuning Guidelines
1. Start with recommended default values
2. Monitor simulation for instability or unrealistic behavior
3. Adjust parameters to balance accuracy and performance needs
4. Validate results against known physical behaviors

## Sources and References

1. Baraff, D. (1997). An introduction to physically based modeling: Constrained dynamics. ACM SIGGRAPH course notes.
2. Stewart, A., & Trinkle, J. (1996). An implicit time-stepping scheme for rigid body dynamics with contact, friction, and constraints. Mathematical Programming.
3. NVIDIA. (2024). Isaac Sim Physics Parameters Guide. NVIDIA Developer Documentation.
4. Open Source Robotics Foundation. (2023). Gazebo Physics Engine Configuration.
5. Featherstone, R. (2008). Rigid Body Dynamics Algorithms. Springer.
6. Tedrake, R. (2009). Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation. MIT Press.
7. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.