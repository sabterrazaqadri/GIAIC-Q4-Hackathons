# Data Model: Physical AI & Humanoid Robotics Modules

## Overview

This document defines the key entities, data structures, and relationships for the Physical AI & Humanoid Robotics textbook modules. The data model supports educational content management, simulation environments, AI perception systems, and Vision-Language-Action (VLA) pipelines.

## Core Entities

### 1. LearningModule
Represents an educational module containing chapters, labs, and assessments with specified learning objectives.

**Fields**:
- moduleId: String (unique identifier, e.g., "module-2")
- title: String (e.g., "The Digital Twin: Simulation Environment")
- description: String (summary of the module's content and purpose)
- learningObjectives: Array<String> (list of learning objectives for the module)
- chapters: Array<Chapter> (related chapters within this module)
- duration: Number (estimated hours to complete the module)
- prerequisites: Array<String> (prerequisites required for this module)
- academicSources: Array<AcademicSource> (citations and references)

### 2. Chapter
Contains technical explanations, hands-on labs, and comprehension checks within a LearningModule.

**Fields**:
- chapterId: String (unique identifier within the module, e.g., "module-2-chapter-1")
- title: String (title of the chapter)
- content: String (main content in Markdown format)
- learningObjectives: Array<String> (specific learning objectives for this chapter)
- labs: Array<LabExercise> (hands-on lab exercises in this chapter)
- comprehensionChecks: Array<ComprehensionCheck> (comprehension checks in this chapter)
- academicCitations: Array<AcademicCitation> (inline citations used in this chapter)
- estimatedTime: Number (estimated minutes to complete the chapter)

### 3. LabExercise
Provides commands and expected output for students to practice concepts in a practical environment.

**Fields**:
- labId: String (unique identifier within the chapter)
- title: String (title of the lab exercise)
- objectives: Array<String> (learning objectives of the lab)
- prerequisites: Array<String> (software/hardware prerequisites for the lab)
- setupInstructions: String (instructions to set up the lab environment)
- procedure: Array<LabStep> (sequence of steps for completing the lab)
- expectedOutput: String (expected results/output of the lab)
- troubleshootingTips: Array<String> (common issues and how to resolve them)
- difficultyLevel: String (e.g., "beginner", "intermediate", "advanced")

### 4. LabStep
A single step within a lab exercise with commands and expected results.

**Fields**:
- stepNumber: Number (order of the step in the procedure)
- description: String (explanation of what the step accomplishes)
- commands: Array<String> (commands to execute in this step)
- expectedResult: String (what the user should expect to see)

### 5. ComprehensionCheck
A quiz or question to validate understanding of the chapter content.

**Fields**:
- checkId: String (unique identifier within the chapter)
- question: String (the question to answer)
- answerOptions: Array<String> (possible answer options, if applicable)
- correctAnswer: String (correct answer)
- explanation: String (explanation of why the answer is correct)
- difficulty: String ("basic", "intermediate", "advanced")

## Simulation and AI Entities

### 6. SimulationEnvironment
Defines the configuration for robot simulation environments.

**Fields**:
- envId: String (unique identifier for the environment)
- name: String (e.g., "Isaac Sim", "Gazebo", "Unity")
- version: String (version of the simulation software)
- configuration: Object (settings for the simulation environment)
- humanoidModels: Array<HumanoidModel> (supported humanoid models in this environment)
- physicsSettings: PhysicsSettings (physics engine configuration)
- sensorConfigurations: Array<SensorConfiguration> (available sensor setups)

### 7. PhysicsSettings
Configuration parameters for physics simulation.

**Fields**:
- gravity: Vector3 (gravity vector in m/s^2)
- timeStep: Number (simulation time step in seconds)
- solverIterations: Number (number of iterations for constraint solving)
- contactSurfaceLayer: Number (surface layer thickness for contacts)

### 8. HumanoidModel
Represents a humanoid robot model in simulation.

**Fields**:
- modelId: String (unique identifier for the robot model)
- name: String (name of the robot, e.g., "Atlas", "Pepper", "NAO")
- urdfPath: String (path to the URDF description file)
- sdfPath: String (path to the SDF description file, if applicable)
- jointCount: Number (number of joints in the model)
- linkCount: Number (number of links in the model)
- sensors: Array<Sensor> (sensors attached to this model)
- actuators: Array<Actuator> (actuators controlling this model)
- mass: Number (total mass in kg)
- dimensions: Dimensions (physical dimensions of the model)

### 9. Dimensions
Physical dimensions of a robot or object.

**Fields**:
- height: Number (height in meters)
- width: Number (width in meters)
- depth: Number (depth in meters)

### 10. Sensor
A sensor attached to a robot model.

**Fields**:
- sensorId: String (unique identifier for the sensor)
- type: String (e.g., "lidar", "camera", "imu", "depth_camera", "force_torque")
- name: String (name identifier for the sensor)
- position: Position3D (position relative to parent link)
- rotation: Rotation3D (orientation relative to parent link)
- parameters: Object (sensor-specific configuration parameters)

### 11. Actuator
An actuator controlling a joint in the robot model.

**Fields**:
- actuatorId: String (unique identifier for the actuator)
- jointName: String (name of the joint this actuator controls)
- type: String (e.g., "position", "velocity", "effort")
- limits: JointLimits (limits for the actuator values)

### 12. JointLimits
Value limits for a joint or actuator.

**Fields**:
- min: Number (minimum allowed value)
- max: Number (maximum allowed value)
- effort: Number (maximum effort/torque limit)
- velocity: Number (maximum velocity limit)

### 13. Position3D
3D position in space.

**Fields**:
- x: Number (x coordinate in meters)
- y: Number (y coordinate in meters)
- z: Number (z coordinate in meters)

### 14. Rotation3D
3D rotation using Euler angles.

**Fields**:
- roll: Number (rotation around x-axis in radians)
- pitch: Number (rotation around y-axis in radians)
- yaw: Number (rotation around z-axis in radians)

## AI and Vision-Language-Action Entities

### 15. PerceptionPipeline
Defines the configuration for AI perception systems.

**Fields**:
- pipelineId: String (unique identifier for the pipeline)
- name: String (name of the pipeline)
- components: Array<PipelineComponent> (components in the pipeline)
- inputSources: Array<String> (sources of input data, e.g., "camera", "lidar")
- outputTypes: Array<String> (types of output data, e.g., "objects", "map")

### 16. PipelineComponent
A component in an AI perception pipeline.

**Fields**:
- componentId: String (unique identifier for the component)
- name: String (name of the component)
- type: String (e.g., "object_detection", "slam", "segmentation")
- configuration: Object (configuration parameters for the component)
- inputs: Array<String> (input data types)
- outputs: Array<String> (output data types)

### 17. VLAPipeline
Vision-Language-Action pipeline that connects speech input to robot actions.

**Fields**:
- pipelineId: String (unique identifier for the VLA pipeline)
- name: String (name of the pipeline)
- speechToText: SpeechToTextConfig (speech recognition configuration)
- languageProcessing: LanguageProcessingConfig (LLM configuration)
- actionValidation: ActionValidationConfig (validation layer configuration)
- rosActions: Array<ROSCall> (possible ROS actions this pipeline can execute)

### 18. SpeechToTextConfig
Configuration for speech-to-text processing.

**Fields**:
- model: String (e.g., "whisper-large-v2")
- device: String (e.g., "cuda", "cpu")
- language: String (language to recognize, e.g., "en")
- sampleRate: Number (audio sampling rate in Hz)

### 19. LanguageProcessingConfig
Configuration for language processing and planning.

**Fields**:
- model: String (e.g., "gpt-4", "gpt-3.5-turbo")
- maxTokens: Number (maximum tokens for processing)
- temperature: Number (creativity parameter)
- systemPrompt: String (system prompt for the LLM)
- actionSpaceDefinition: String (definition of available actions)

### 20. ActionValidationConfig
Configuration for validating robot actions before execution.

**Fields**:
- safetyConstraints: Array<String> (list of safety constraints to check)
- actionWhitelist: Array<String> (allowed actions)
- simulationMode: Boolean (whether to run in simulation only)
- confirmationRequired: Boolean (whether human confirmation is required)

### 21. ROSCall
A representation of a ROS service call or action.

**Fields**:
- callId: String (unique identifier for the call)
- serviceName: String (name of the ROS service)
- actionType: String (type of action, e.g., "move_base", "pick_object")
- parameters: Object (parameters for the service call)
- timeout: Number (timeout for the service call in seconds)

## Academic and Citation Entities

### 22. AcademicSource
A source for academic citations.

**Fields**:
- sourceId: String (unique identifier for the source)
- title: String (title of the publication)
- authors: Array<String> (authors of the publication)
- publicationDate: String (date in YYYY-MM-DD format)
- publisher: String (publisher name)
- url: String (URL to the publication)
- doi: String (DOI if available)
- sourceType: String (e.g., "journal", "conference", "book", "technical_report")

### 23. AcademicCitation
A citation used within the textbook.

**Fields**:
- citationId: String (unique identifier for the citation)
- source: AcademicSource (the source being cited)
- citationText: String (the text of the citation)
- pageReference: String (optional page reference)
- context: String (the context in which the citation is used)

## Relationships

### Module → Chapter
- One LearningModule has many Chapters
- Defined by the `chapters` array property in LearningModule

### Chapter → LabExercise
- One Chapter has many LabExercises
- Defined by the `labs` array property in Chapter

### Chapter → ComprehensionCheck
- One Chapter has many ComprehensionChecks
- Defined by the `comprehensionChecks` array property in Chapter

### SimulationEnvironment → HumanoidModel
- One SimulationEnvironment supports many HumanoidModels
- Defined by the `humanoidModels` array property in SimulationEnvironment

### HumanoidModel → Sensor
- One HumanoidModel has many Sensors
- Defined by the `sensors` array property in HumanoidModel

### HumanoidModel → Actuator
- One HumanoidModel has many Actuators
- Defined by the `actuators` array property in HumanoidModel

### PerceptionPipeline → PipelineComponent
- One PerceptionPipeline has many PipelineComponents
- Defined by the `components` array property in PerceptionPipeline

### VLAPipeline → ROSCall
- One VLAPipeline can execute many ROSCalls
- Defined by the `rosActions` array property in VLAPipeline