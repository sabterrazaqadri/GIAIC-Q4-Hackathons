# Physics Parameters for Humanoid Simulation

## Overview
This document provides research on the essential physics parameters for humanoid robot simulation in environments like Isaac Sim and Gazebo. These parameters are crucial for achieving realistic simulation behavior and ensuring proper sim-to-real transfer.

## Key Physics Concepts for Humanoid Simulation

### 1. Gravity
- **Standard Value**: 9.81 m/s² on Earth
- **Configuration**: Applied as a constant downward force
- **Considerations**: 
  - Small variations possible for different celestial bodies
  - Critical for realistic motion and balance simulation
  - Affects all dynamic interactions in the simulation

### 2. Time Step (dt)
- **Definition**: The time interval between physics simulation updates
- **Typical Range**: 0.001s to 0.01s (1ms to 10ms)
- **Impact**:
  - Smaller steps = more accurate but computationally expensive
  - Larger steps = less accurate but faster simulation
  - Must be small enough to capture fast dynamics in humanoid systems

### 3. Solver Settings
- **Solver Type**: 
  - Sequential Impulse (common in game engines)
  - Projected Gauss-Seidel (PGS)
  - TGS (Two Gauss-Seidel) - NVIDIA's choice in Isaac Sim
- **Iterations**: Number of solver iterations affects stability
  - Higher iterations = more stable but slower
  - Position iterations (typically 4-8 for humanoid)
  - Velocity iterations (typically 1-4)

### 4. Material Properties

#### Static Friction
- **Definition**: Resistance to initial motion between surfaces
- **Typical Range**: 0.1 to 1.0 for common materials
- **Humanoid Considerations**: 
  - Feet interacting with ground surfaces
  - Important for walking and balance simulation

#### Dynamic Friction
- **Definition**: Resistance to continued motion between surfaces
- **Typical Range**: 0.1 to 0.8 (usually less than static friction)
- **Humanoid Considerations**:
  - Sliding motions during movement
  - Different from static friction (usually ~80% of static)

#### Restitution (Bounciness)
- **Definition**: Energy retention during collisions (0.0 = no bounce, 1.0 = perfect bounce)
- **Typical Range**: 0.0 to 0.5 for humanoid applications
- **Humanoid Considerations**:
  - Low values for realistic foot-ground interactions
  - Higher values for ball-jointed robots or special cases

## Physics Parameters for Humanoid Robots

### Mass Properties
- **Total Robot Mass**: Typically 20-100kg depending on size
- **Link Masses**: Distributed according to actual robot design
- **Center of Mass**: Critical for balance and stability
- **Inertial Tensors**: Define how mass is distributed around each link

### Joint Parameters
- **Joint Limits**: Position, velocity, and effort limits
- **Damping**: Resistance to motion (typically 0.1-1.0)
- **Friction**: Static and dynamic friction coefficients
- **Spring Constants**: For compliant joints (if applicable)

### Contact Parameters
- **Contact Points**: Where robot interacts with environment
- **Contact Stiffness**: How rigid the contact appears (typically high)
- **Contact Damping**: Energy absorption during contact
- **Rest Offset**: Distance at which contact response begins
- **Contact Surface Layer**: Small buffer to prevent interpenetration

## Recommended Parameters for Isaac Sim

```yaml
physics:
  solver_type: "TGS"  # Two Gauss-Seidel solver
  num_position_iterations: 8
  num_velocity_iterations: 2
  enable_stabilization: true
  stabilization_threshold: 0.001
  
  # Contact settings
  contact_offset: 0.002  # 2mm
  rest_offset: 0.0
  bounce_threshold: 0.5  # velocity threshold for bounce
  
  # Material properties
  default_static_friction: 0.5
  default_dynamic_friction: 0.4
  default_restitution: 0.1
  
  # Articulation settings for humanoid joints
  articulation_position_iteration: 4
  articulation_velocity_iteration: 1
```

## Recommended Parameters for Gazebo

```sdf
<physics type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver type="quick" iters="10" sor="1.0" />
    <constraints cfm="0.0" erp="0.2" 
                 contact_max_correcting_vel="0.1" 
                 contact_surface_layer="0.001" />
  </ode>
</physics>
```

## Sim-to-Real Considerations

### Parameter Tuning
- Start with realistic values based on physical measurements
- Calibrate on real robot to match behavior
- Account for differences in actuator dynamics
- Consider sensor noise and delay in simulation

### Validation Techniques
- Compare kinematic behavior between sim and real
- Validate dynamic responses (e.g., walking patterns)
- Test control strategies in both environments
- Measure sim-to-real performance gaps

## Sources and References

1. Featherstone, R. (2008). Rigid Body Dynamics Algorithms. Springer.
2. Mordatch, I., et al. (2012). Discovery of complex behaviors through contact-invariant optimization. ACM Transactions on Graphics.
3. Tedrake, R. (2009). Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation. MIT Press.
4. NVIDIA. (2024). Isaac Sim Physics Simulation Guide. NVIDIA Developer Documentation.
5. Open Source Robotics Foundation. (2023). Gazebo Physics Engine Documentation.
6. Khatib, O., et al. (2018). Human-Centered Robotics. Springer Handbook of Robotics.
7. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.