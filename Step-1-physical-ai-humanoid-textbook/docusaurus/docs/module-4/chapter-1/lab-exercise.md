---
title: "Lab Exercise 1: Implementing Speech-to-Text Conversion"
sidebar_position: 2
---

# Lab Exercise 1: Implementing Speech-to-Text Conversion

## Objectives

In this lab exercise, you will:

1. Set up an audio capture system to record voice commands
2. Integrate OpenAI's Whisper API for speech-to-text conversion
3. Create a ROS 2 node that processes audio and publishes transcribed text
4. Test the system with various voice commands

## Prerequisites

Before starting this lab, ensure you have:

- A working ROS 2 Humble Hawksbill installation
- Python 3.8 or higher
- A microphone connected to your computer
- An OpenAI API key for using Whisper
- Basic understanding of ROS 2 concepts (nodes, topics, services)

## Estimated Time

This lab should take approximately 45-60 minutes to complete.

## Setup Instructions

1. Create a new ROS 2 package for this lab:
   ```bash
   cd ~/ros2_ws/src
   ros2 pkg create --build-type ament_python speech_to_text_lab --dependencies rclpy std_msgs builtin_interfaces
   cd speech_to_text_lab
   ```

2. Create the following directory structure:
   ```
   speech_to_text_lab/
   ├── speech_to_text_lab/
   │   ├── __init__.py
   │   ├── audio_capture_node.py
   │   ├── whisper_node.py
   │   └── command_interpreter_node.py
   ├── test/
   ├── setup.cfg
   ├── setup.py
   └── package.xml
   ```

3. Install additional Python dependencies:
   ```bash
   pip install openai pyaudio speechrecognition
   ```

4. Set your OpenAI API key as an environment variable:
   ```bash
   export OPENAI_API_KEY="your-api-key-here"
   ```

## Procedure

### Step 1: Implement Audio Capture Node

Create the audio capture node that will record audio from your microphone:

1. In the `speech_to_text_lab/speech_to_text_lab/` directory, create `audio_capture_node.py`

2. Add the following code to implement audio capture:

```python
import rclpy
from rclpy.node import Node
import pyaudio
import wave
import threading
import queue
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import time

class AudioCaptureNode(Node):
    def __init__(self):
        super().__init__('audio_capture_node')
        
        # Audio configuration
        self.chunk = 1024
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 44100
        self.record_seconds = 5  # Record for 5 seconds at a time
        
        # Initialize pyaudio
        self.p = pyaudio.PyAudio()
        
        # Publisher to send audio file paths
        self.audio_path_pub = self.create_publisher(String, 'audio_path', 10)
        
        # Timer to trigger continuous recording
        self.timer = self.create_timer(6.0, self.record_audio)  # Record every 6 seconds
        
        self.get_logger().info("Audio Capture Node initialized")
    
    def record_audio(self):
        """Record audio and save to file"""
        stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        
        self.get_logger().info("Recording audio...")
        frames = []
        
        for i in range(0, int(self.rate / self.chunk * self.record_seconds)):
            data = stream.read(self.chunk)
            frames.append(data)
            
        self.get_logger().info("Recording finished.")
        
        # Stop stream and close
        stream.stop_stream()
        stream.close()
        
        # Generate a unique filename
        timestamp = time.time()
        filename = f"recording_{int(timestamp)}.wav"
        
        # Save to WAV file
        with wave.open(filename, 'wb') as wf:
            wf.setnchannels(self.channels)
            wf.setsampwidth(self.p.get_sample_size(self.format))
            wf.setframerate(self.rate)
            wf.writeframes(b''.join(frames))
        
        self.get_logger().info(f"Audio saved to {filename}")
        
        # Publish the file path
        msg = String()
        msg.data = filename
        self.audio_path_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up resources"""
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    audio_node = AudioCaptureNode()
    
    try:
        rclpy.spin(audio_node)
    except KeyboardInterrupt:
        pass
    finally:
        audio_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 2: Implement Whisper Processing Node

Create the node that will use Whisper to convert audio to text:

1. In the same directory, create `whisper_node.py`

2. Add the following code:

```python
import rclpy
from rclpy.node import Node
import openai
import os
from std_msgs.msg import String
from builtin_interfaces.msg import Time

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        
        # Set your OpenAI API key
        openai.api_key = os.getenv("OPENAI_API_KEY")
        
        # Subscriber to audio file paths
        self.audio_sub = self.create_subscription(
            String,
            'audio_path',
            self.audio_callback,
            10
        )
        
        # Publisher for transcribed text
        self.text_pub = self.create_publisher(String, 'transcribed_text', 10)
        
        self.get_logger().info("Whisper Node initialized")
    
    def transcribe_audio(self, audio_file_path):
        """Transcribe audio file using Whisper"""
        try:
            self.get_logger().info(f"Transcribing: {audio_file_path}")
            with open(audio_file_path, "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
            return transcript.text
        except Exception as e:
            self.get_logger().error(f"Error transcribing audio: {e}")
            return ""
    
    def audio_callback(self, msg):
        """Callback function when audio path is received"""
        audio_path = msg.data
        
        # Transcribe the audio
        transcript = self.transcribe_audio(audio_path)
        
        if transcript:
            self.get_logger().info(f"Transcribed text: {transcript}")
            
            # Publish the transcribed text
            text_msg = String()
            text_msg.data = transcript
            self.text_pub.publish(text_msg)
        else:
            self.get_logger().warn(f"Failed to transcribe {audio_path}")

def main(args=None):
    rclpy.init(args=args)
    
    whisper_node = WhisperNode()
    
    try:
        rclpy.spin(whisper_node)
    except KeyboardInterrupt:
        pass
    finally:
        whisper_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create a Node to Interpret Commands

Create a simple command interpreter to simulate using the transcribed text:

1. Create `command_interpreter_node.py`

2. Add the following code:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandInterpreterNode(Node):
    def __init__(self):
        super().__init__('command_interpreter_node')
        
        # Subscriber to transcribed text
        self.text_sub = self.create_subscription(
            String,
            'transcribed_text',
            self.text_callback,
            10
        )
        
        # Publisher for processed commands
        self.command_pub = self.create_publisher(String, 'robot_command', 10)
        
        self.get_logger().info("Command Interpreter Node initialized")
    
    def text_callback(self, msg):
        """Callback function when transcribed text is received"""
        text = msg.data.lower()  # Convert to lowercase for processing
        
        self.get_logger().info(f"Processing text: {text}")
        
        # Simple command recognition
        if "move forward" in text:
            command = "MOVE_FORWARD"
        elif "turn left" in text or "turn to the left" in text:
            command = "TURN_LEFT"
        elif "turn right" in text or "turn to the right" in text:
            command = "TURN_RIGHT"
        elif "stop" in text:
            command = "STOP"
        elif "hello" in text or "hi " in text:
            command = "GREETING"
        else:
            command = "UNKNOWN_COMMAND"
        
        self.get_logger().info(f"Interpreted command: {command}")
        
        # Publish the interpreted command
        cmd_msg = String()
        cmd_msg.data = command
        self.command_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    
    interpreter_node = CommandInterpreterNode()
    
    try:
        rclpy.spin(interpreter_node)
    except KeyboardInterrupt:
        pass
    finally:
        interpreter_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Update setup.py

Update the `setup.py` file to make your nodes executable:

```python
from setuptools import find_packages, setup

package_name = 'speech_to_text_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A package for speech to text conversion in robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_capture = speech_to_text_lab.audio_capture_node:main',
            'whisper_node = speech_to_text_lab.whisper_node:main',
            'command_interpreter = speech_to_text_lab.command_interpreter_node:main',
        ],
    },
)
```

### Step 5: Build and Test the Package

1. Build your package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select speech_to_text_lab
   source install/setup.bash
   ```

2. Open three separate terminals and run each node:

   Terminal 1:
   ```bash
   ros2 run speech_to_text_lab audio_capture
   ```

   Terminal 2:
   ```bash
   ros2 run speech_to_text_lab whisper_node
   ```

   Terminal 3:
   ```bash
   ros2 run speech_to_text_lab command_interpreter
   ```

3. Speak into your microphone and observe the transcription and command interpretation in the terminals.

## Expected Output

When you run the system and speak into the microphone, you should see:

1. In the audio capture terminal: "Recording audio...", "Recording finished.", and "Audio saved to recording_*.wav"
2. In the whisper terminal: "Transcribing: recording_*.wav", "Transcribed text: [your spoken words]", etc.
3. In the command interpreter terminal: "Processing text: [your spoken words]", "Interpreted command: [appropriate command]"

## Troubleshooting Tips

1. **Audio Recording Issues**: Check that your microphone is properly connected and configured as the default input device.

2. **OpenAI API Issues**: 
   - Verify that your `OPENAI_API_KEY` environment variable is set correctly
   - Check that you have sufficient credits in your OpenAI account

3. **ROS 2 Topic Communication Issues**:
   - Use `ros2 topic list` to verify that topics are being published
   - Use `ros2 topic echo` to check the content of messages

4. **Python Dependencies**:
   - Make sure all required packages are installed (`pip install openai pyaudio speechrecognition`)
   - Verify that you're using a compatible Python version

## Extensions (Optional)

1. Add error handling for audio file cleanup after transcription
2. Implement a wake word detection to trigger recording only when the robot's name is mentioned
3. Add audio preprocessing to improve transcription quality
4. Implement confidence scoring for transcriptions and only process high-confidence results

## Summary

In this lab, you've successfully implemented a speech-to-text system using OpenAI's Whisper API integrated with ROS 2. The system captures audio, converts it to text, and interprets the text as robot commands. This forms a foundational component of the Vision-Language-Action pipeline, enabling natural interaction with robotic systems.

In the next chapter, we'll build on this foundation by implementing Large Language Model-based task planning to convert the transcribed speech into sequences of robot actions.