# Simulation Scenario: Basic Humanoid Physics

## Overview
This document describes a simulation scenario for testing basic humanoid physics in Isaac Sim or Gazebo. The scenario involves a simple humanoid model performing basic physical interactions like balancing, walking in place, and manipulating objects.

## Scenario Requirements

### Humanoid Model
For this scenario, we'll use a simplified humanoid model with:
- 20+ degrees of freedom (DoF)
- Basic anthropomorphic structure:
  - Head, torso, pelvis
  - 2 arms with shoulder, elbow, and wrist joints
  - 2 legs with hip, knee, and ankle joints
- Appropriate mass distribution
- Collision geometry for each link

### Environment Setup
- Flat ground plane with appropriate friction
- Several objects for interaction (cubes of different sizes and masses)
- Lighting for visual comfort
- Basic obstacles to navigate around

## Scenario Objectives

### 1. Balance and Stability Test
- **Objective**: Demonstrate basic balance capabilities
- **Setup**: 
  - Spawn humanoid model in standing position
  - Apply small external forces periodically
- **Expected Behavior**: 
  - Model should maintain balance using ankle and hip strategies
  - Center of mass remains within support polygon

### 2. Walking in Place
- **Objective**: Demonstrate basic locomotion physics
- **Setup**:
  - Initial standing position
  - Cyclic joint commands to simulate walking motion
- **Expected Behavior**:
  - Natural walking gait emerges
  - Proper foot-ground contact and lift-off
  - Stable balance during double and single support phases

### 3. Simple Object Manipulation
- **Objective**: Demonstrate interaction physics
- **Setup**:
  - Place small lightweight object in front of humanoid
  - Control system commands to reach and grasp
- **Expected Behavior**:
  - Realistic reaching motion
  - Proper contact forces when touching object
  - Physically plausible lifting motion

## Simulation Configuration

### Physics Settings
```yaml
physics:
  gravity: [0.0, 0.0, -9.81]  # Standard Earth gravity
  time_step: 0.001  # 1ms for high fidelity
  solver_type: "TGS"  # Two Gauss-Seidel for Isaac Sim
  position_iterations: 8
  velocity_iterations: 2
  contact_offset: 0.002  # 2mm
  rest_offset: 0.0
  default_static_friction: 0.5
  default_dynamic_friction: 0.4
  default_restitution: 0.1
```

### Humanoid Model Properties
- **Total Mass**: ~50 kg (adjustable based on specific model)
- **Height**: ~1.5m (for small humanoid, adjustable)
- **Link Masses**: Distributed according to human-like proportions
- **Inertial Properties**: Calculated from CAD models or approximated
- **Joint Limits**: Based on human biomechanics
- **Damping**: Appropriate values to prevent oscillation

### Environmental Properties
- **Ground Friction**: Static=0.8, Dynamic=0.7 (for good grip)
- **Object Masses**: 
  - Small cube: 0.1 kg
  - Medium cube: 0.5 kg
  - Large cube: 1.0 kg
- **Object Materials**: Plastic-like with appropriate friction values

## Implementation Steps

### Step 1: Environment Creation
1. Create ground plane with appropriate physics properties
2. Add humanoid model to the scene
3. Position objects for manipulation test
4. Configure lighting and camera

### Step 2: Physics Parameter Configuration
1. Set gravity to standard value
2. Configure time step for stability
3. Set solver parameters for adequate accuracy
4. Configure contact properties

### Step 3: Control System Integration (Optional)
1. Implement basic balance controller (if testing active behaviors)
2. Create trajectory generator for walking pattern
3. Implement reaching/grasping controller

### Step 4: Scenario Execution
1. Initialize humanoid in default pose
2. Execute balance test with external perturbations
3. Run walking in place pattern
4. Perform object manipulation sequence
5. Record and analyze results

## Expected Results

### Balance Test Results
- Humanoid maintains upright position under small perturbations
- Joint torques remain within realistic bounds
- Center of pressure stays within feet support area

### Walking Simulation Results
- Stable periodic gait emerges
- No foot slippage or unrealistic ground penetration
- Natural looking motion patterns

### Manipulation Results
- Successful reaching with smooth trajectories
- Proper force application during contact
- Stable lifting motion without dropping object

## Troubleshooting Common Issues

### Instability Problems
- **Symptoms**: Oscillations, jittering, explosion of simulation
- **Solutions**: Increase solver iterations, reduce time step, check mass properties

### Penetration Issues
- **Symptoms**: Objects passing through each other
- **Solutions**: Reduce time step, increase contact stiffness, verify collision geometry

### Controller Issues
- **Symptoms**: Unstable behavior when control is applied
- **Solutions**: Verify control frequency matches physics frequency, check joint limits

## Validation Metrics

### Balance Metrics
- Center of Mass (CoM) position relative to feet
- Zero Moment Point (ZMP) location
- Joint space deviations from nominal pose

### Walking Metrics
- Step length and frequency
- Ground contact timing
- Energy efficiency (estimated)

### Manipulation Metrics
- Trajectory tracking error
- Contact force magnitudes
- Grasp stability

## Extensions

### Advanced Variations
1. **Sloped Terrain**: Test balance on inclined surfaces
2. **Multiple Objects**: Complex manipulation tasks
3. **Dynamic Obstacles**: Moving objects to avoid
4. **Different Surfaces**: Ice, sand, gravel for different friction

### Data Collection
1. Joint positions, velocities, and torques
2. Center of mass trajectory
3. Contact forces and locations
4. Simulation timing and performance metrics

This simulation scenario provides a comprehensive testbed for validating the physics parameters of humanoid robots in simulation environments like Isaac Sim and Gazebo.