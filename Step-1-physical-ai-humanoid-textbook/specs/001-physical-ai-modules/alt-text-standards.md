# Alt-text Standards for Diagrams and Visual Elements

## Purpose

Alt-text (alternative text) provides textual descriptions of images, diagrams, and other visual elements. This is crucial for accessibility, allowing screen readers to convey visual information to visually impaired users. This document establishes standards for creating effective alt-text for diagrams in the Physical AI & Humanoid Robotics textbook.

## General Principles

1. **Descriptive but Concise**: Alt-text should accurately describe the content and function of the image without being unnecessarily verbose
2. **Context-Aware**: Alt-text should be appropriate to the content in which the image appears
3. **Functional Focus**: Focus on what the image shows and its relevance to the text
4. **Avoid Redundancy**: Don't repeat information already provided in surrounding text

## Alt-text Standards by Diagram Type

### Architecture Diagrams
```
Example: Diagram showing the Vision-Language-Action pipeline architecture
Alt-text: "Architecture diagram showing the Vision-Language-Action pipeline. Audio input connects to Whisper speech-to-text, which connects to the LLM for planning, which connects to ROS 2 action clients for robot control. Validation layers are positioned between the LLM and ROS 2 actions for safety."
```

### Simulation Environment Screenshots
```
Example: Screenshot of Isaac Sim environment
Alt-text: "Screenshot of Isaac Sim environment showing a humanoid robot model in a living room setting with furniture. The robot is positioned near a couch with a coffee table, and has a LiDAR sensor mounted on its head."
```

### Data Flow Diagrams
```
Example: Diagram showing data flow in perception pipeline
Alt-text: "Data flow diagram for the perception pipeline. Camera data flows to image processing, then to object detection, then to object recognition. LiDAR data flows to point cloud processing, then to SLAM mapping, then to path planning."
```

### Process Flowcharts
```
Example: Figure showing the sim-to-real transfer process
Alt-text: "Process flowchart showing sim-to-real transfer. Begins with simulation environment, followed by synthetic data generation, domain randomization, real-world validation, and concludes with deployment to physical robot."
```

### Graphs and Charts
```
Example: Performance comparison graph
Alt-text: "Bar chart comparing simulation performance across different platforms. Isaac Sim shows 60 FPS, Gazebo Garden shows 45 FPS, and Unity shows 50 FPS for humanoid robot simulation with 10 sensors active."
```

## Technical Considerations

### Length Guidelines
- **Simple images**: 1-2 phrases (under 15 words)
- **Complex images**: Complete sentences (under 100 words)
- **Detailed diagrams**: Brief paragraphs (under 200 words)

### Special Cases
- **Decorative images**: Use empty alt-text (`alt=""`)
- **Charts with many data points**: Include a reference to a table with detailed data
- **Complex multi-layered diagrams**: Provide a link to a detailed description

## Examples of Good vs Poor Alt-text

### Poor: Too Vague
```
<img src="isaac-sim.png" alt="Isaac Sim screenshot">
```

### Good: Descriptive
```
<img src="isaac-sim.png" alt="Screenshot of Isaac Sim environment showing a humanoid robot model in a living room setting with furniture. The robot is positioned near a couch with a coffee table, and has a LiDAR sensor mounted on its head.">
```

### Poor: Too Verbose
```
<img src="vla-arch.png" alt="This diagram shows the Vision-Language-Action pipeline used in robotics applications. On the left side, we have audio input which goes to Whisper speech-to-text engine. The Whisper engine converts the audio to text and sends it to the Large Language Model. The LLM creates a plan based on the text input and sends it to the ROS 2 action clients which then execute the plan on the robot.">
```

### Good: Concise but Complete
```
<img src="vla-arch.png" alt="Architecture diagram showing the Vision-Language-Action pipeline. Audio input connects to Whisper speech-to-text, which connects to the LLM for planning, which connects to ROS 2 action clients for robot control. Validation layers are positioned between the LLM and ROS 2 actions for safety.">
```

## Implementation Guidelines

1. **All diagrams must have alt-text** - no exceptions for accessibility compliance
2. **Review by diverse team** - have both technical and accessibility-focused team members review alt-text
3. **Test with screen readers** - periodically test content with accessibility tools
4. **Update when diagrams change** - maintain alt-text accuracy when diagrams are updated
5. **Placeholder for complex diagrams** - for very complex diagrams that require more than 200 words, use alt-text that summarizes the key components and include a detailed description in the figure caption or a linked section

## Quality Assurance Checklist

- [ ] Alt-text accurately describes the image
- [ ] Alt-text is concise but comprehensive
- [ ] Alt-text is appropriate for the context
- [ ] Alt-text does not duplicate surrounding text
- [ ] Alt-text follows accessibility standards
- [ ] Complex diagrams have detailed descriptions when needed