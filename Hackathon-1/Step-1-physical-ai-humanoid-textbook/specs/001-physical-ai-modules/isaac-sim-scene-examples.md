# Isaac Sim Scene Configuration Examples and Best Practices

## Overview
This document provides research on Isaac Sim scene configuration examples and best practices for creating simulation scenes for humanoid robotics. This research will support Module 3.1 content on advanced simulation scenarios.

## Isaac Sim Scene Architecture

### Core Components
- **World**: Root element containing all scene elements
- **Lights**: Environmental lighting (Distant, Dome, Point, etc.)
- **Cameras**: Viewpoints for rendering and sensor simulation
- **Static Objects**: Non-moving environmental elements
- **Articulated Bodies**: Robots and movable objects with joints
- **Materials**: Surface properties for rendering and physics
- **USD Primitives**: Universal Scene Description elements

### Scene Organization
Isaac Sim uses Universal Scene Description (USD) as its core scene format. Scenes are hierarchically organized with transforms applying to child objects.

## Isaac Sim Scene Configuration Examples

### 1. Basic Humanoid Environment
```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.prims import create_primitive
from omni.isaac.core.utils.light import create_distant_light
from omni.isaac.core.utils.camera import create_camera

# Create a new world
world = World(stage_units_in_meters=1.0)

# Add lighting
create_distant_light("/World/DistantLight", intensity=3000)

# Add ground plane
create_primitive(
    prim_path="/World/GroundPlane",
    primitive_props={"size": 100.0},
    position=[0, 0, 0],
    orientation=[0, 0, 0, 1],
    scale=[1, 1, 1],
    visible=True
)

# Add a simple humanoid model (placeholder - actual model path varies)
add_reference_to_stage(
    usd_path="path/to/humanoid_model.usd",
    prim_path="/World/Humanoid"
)

# Add a camera for perception
camera = create_camera(
    prim_path="/World/Camera",
    position=[3.0, 0.0, 1.5],
    orientation=[0.0, 0.0, 0.0, 1.0]
)
```

### 2. Advanced Scene with Complex Environment
```python
import omni
from pxr import UsdGeom, Usd, Gf
import carb

# Create a more complex scene
stage = omni.usd.get_context().get_stage()

# Create a room with walls
room_x = 5.0
room_y = 5.0
room_z = 3.0

# Create room dimensions as a reference
room_prim = UsdGeom.Xform.Define(stage, "/World/Room")

# Add floor
floor_prim = UsdGeom.Mesh.Define(stage, "/World/Room/Floor")
# (Code to define floor mesh using vertices, faces, etc.)

# Add walls
wall_thickness = 0.1
walls = [
    {"name": "FrontWall", "pos": [0, room_y/2, room_z/2], "size": [room_x, wall_thickness, room_z]},
    {"name": "BackWall", "pos": [0, -room_y/2, room_z/2], "size": [room_x, wall_thickness, room_z]},
    {"name": "LeftWall", "pos": [-room_x/2, 0, room_z/2], "size": [wall_thickness, room_y, room_z]},
    {"name": "RightWall", "pos": [room_x/2, 0, room_z/2], "size": [wall_thickness, room_y, room_z]}
]

for wall in walls:
    wall_prim = UsdGeom.Cube.Define(stage, f"/World/Room/{wall['name']}")
    wall_prim.CreateSizeAttr(wall['size'])
    wall_prim.AddTranslateOp().Set(Gf.Vec3d(*wall['pos']))
```

### 3. Perception-Ready Scene Configuration
```python
from omni.isaac.sensor import Camera
import omni.replicator.core as rep

# Enable Isaac Replicator for synthetic data generation
rep.orchestrator._orchestrator.start_async()

# Create a perception-ready camera
camera = Camera(
    prim_path="/World/Camera",
    position=[2.0, 2.0, 1.5],
    frequency=20
)

# Register sensors for synthetic data generation
with rep.trigger.on_frame(num_zeros=100):
    # Generate segmentation labels
    semantic_instance = rep.annotators.cheat_semantic_segmentation()
    
    # Generate depth data
    depth_data = rep.annotators.cheat_depth(max_depth=10)
    
    # Generate bounding boxes
    bbox_2d_tight = rep.annotators.cheat_bb2d_tight()
    
    # Generate poses
    pose_data = rep.annotators.cheat_camera_poses()
```

### 4. Physics Optimization Configuration
```python
from omni.isaac.core import PhysicsContext

# Set physics parameters for humanoid simulation
physics_ctx = PhysicsContext()
physics_ctx.set_physics_dt(1.0/60.0)  # 60Hz physics update
physics_ctx.set_rendering_dt(1.0/30.0)  # 30Hz rendering update

# Configure for humanoid stability
physics_ctx.set_gravity([0.0, 0.0, -9.81])
physics_ctx.set_solver_type("TGS")  # Two Gauss-Seidel
physics_ctx.set_num_position_iterations(8)
physics_ctx.set_num_velocity_iterations(2)
```

## Best Practices for Simulation Scene Creation

### 1. Performance Optimization
- **Keep polygon counts reasonable**: High-polygon objects can significantly slow simulation
- **Use instancing for repeated objects**: Such as chairs in a room or trees in an outdoor scene
- **Optimize lighting**: Use distant lights instead of point lights when possible
- **Limit dynamic objects**: Only make objects dynamic if they need to be

### 2. Reusability and Modularity
- **Organize scenes hierarchically**: Group related objects together
- **Use USD layering**: Separate static environment from dynamic elements
- **Create template scenes**: For consistent starting points across experiments
- **Parameterize scenes**: Use configuration files for changing scene parameters

### 3. Physics Considerations
- **Accurate mass properties**: Ensure all objects have realistic mass and inertial properties
- **Appropriate friction values**: Set coefficient of friction based on materials
- **Stable joint configurations**: Avoid configurations that might cause simulation instability
- **Collision mesh optimization**: Use detailed meshes for rendering, simplified for collision

### 4. Sensor Readiness
- **Clear sight lines**: Ensure sensors have unobstructed views of areas of interest
- **Appropriate lighting**: Avoid overexposed or underexposed areas
- **Texture variety**: Include surfaces with different textures for visual feature extraction
- **Calibration parameters**: Include known calibration information for synthetic sensors

### 5. Documentation and Reproducibility
- **Version control**: Keep USD files under version control
- **Configuration files**: Store scene parameters in separate config files
- **Metadata**: Include information about scene content and purpose
- **Modular components**: Break scenes into reusable components

## Advanced Isaac Sim Features for Humanoid Robotics

### 1. ROS 2 Bridge Configuration
```python
from omni.isaac.ros_bridge import ROSBridge
import omni.isaac.ros_bridge.helpers as rb_helper

# Set up ROS 2 bridge for humanoid control
rb_helper.set_carb_setting("enable_topic", "/isaac_sim/occupancy_grid")
rb_helper.set_carb_setting("enable_topic", "/isaac_sim/laserscan")
rb_helper.set_carb_setting("frequency", 30.0)
```

### 2. Domain Randomization
```python
import omni.replicator.core as rep

# Randomize lighting conditions
with rep.new_layer():
    lights = rep.get.light()
    
    with lights.randomizer:
        rep.modify.visibility(rep.random_actor(False, True))
        rep.modify.intensity(rep.distribution.normal(1000, 200))
        rep.modify.position(rep.distribution.uniform((-5, -5, 3), (5, 5, 5)))

# Randomize object appearances
with rep.new_layer():
    objects = rep.get.prims(path_pattern="*/RandomizedObject*")
    
    with objects.randomizer:
        rep.randomizer.material(rep.USDShadeMdl.mdl_files, "OmniPBR")
        rep.randomizer.lighting_condition(
            intensity=rep.distribution.uniform(500, 2000),
            color=rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))
        )
```

## Common Patterns and Templates

### 1. Indoor Navigation Scene Template
- Ground plane with appropriate friction
- Walls/obstacles arranged in navigation pattern
- Lighting setup for consistent perception
- Wayfinding markers for navigation tasks
- Humanoid spawn location with clear area

### 2. Object Manipulation Scene Template
- Table surface at appropriate height
- Objects for manipulation with varying properties
- Clear space for robot access
- Overhead camera for task monitoring
- Storage bins for organized scenes

### 3. Human-Robot Interaction Scene Template
- Social interaction space (4-12 ft radius)
- Appropriate furniture (chairs, tables)
- Clear pathways for movement
- Multiple viewpoints for observation
- Safety boundaries for collision avoidance

## Troubleshooting Common Issues

### 1. Performance Problems
- **Issue**: Slow frame rates during simulation
- **Solution**: Reduce polygon count, limit dynamic objects, optimize lighting

### 2. Physics Instability
- **Issue**: Objects jittering or exploding
- **Solution**: Increase solver iterations, check mass properties, adjust time step

### 3. Sensor Malfunction
- **Issue**: Incorrect sensor data
- **Solution**: Verify sensor placement, check calibration, ensure no obstructions

## References and Resources

1. NVIDIA. (2024). Isaac Sim Documentation - Scene Creation. NVIDIA Developer.
2. NVIDIA. (2024). Universal Scene Description Guide. NVIDIA Developer.
3. NVIDIA. (2024). Isaac Sim Best Practices Guide. NVIDIA Developer.
4. Pixar Animation Studios. (2023). Universal Scene Description Specification.
5. Khatib, O., et al. (2018). Human-Centered Robotics: A Designer's Guide. Springer.
6. Siciliano, B., & Khatib, O. (2016). Springer Handbook of Robotics. Springer.
7. Muratore, L., et al. (2021). Domain Randomization for Transfer Learning of Robot Policies. IEEE Robotics and Automation Letters.