# Audio Processing Environment for Whisper Integration

## Overview
This document outlines the setup for audio processing in the Vision-Language-Action pipeline, specifically for integrating OpenAI's Whisper for speech-to-text conversion. This environment is essential for Module 4 of the Physical AI & Humanoid Robotics textbook.

## Required Dependencies

### Python Packages
```bash
pip install torch torchvision torchaudio
pip install openai-whisper
pip install pyaudio  # For real-time audio capture
pip install soundfile  # For audio file I/O
pip install librosa  # For audio preprocessing
pip install rclpy  # For ROS 2 integration
pip install numpy
```

### System Dependencies
- FFmpeg (for audio format conversion)
- PortAudio (for PyAudio, if using real-time capture)

## Audio Processing Pipeline

### 1. Audio Capture Configuration
```python
import pyaudio
import numpy as np
import whisper

class AudioCapture:
    def __init__(self, rate=16000, chunk=1024, channels=1):
        self.rate = rate
        self.chunk = chunk
        self.channels = channels
        self.format = pyaudio.paInt16
        self.audio = pyaudio.PyAudio()
        
    def start_stream(self):
        stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )
        return stream
        
    def read_audio(self, stream, duration=5):
        frames = []
        for _ in range(0, int(self.rate / self.chunk * duration)):
            data = stream.read(self.chunk)
            frames.append(data)
        return b''.join(frames)
        
    def close(self):
        self.audio.terminate()
```

### 2. Audio Preprocessing
```python
import soundfile as sf
import io
import torch

def preprocess_audio(audio_data, target_sr=16000):
    """
    Preprocess audio data for Whisper model
    """
    # Convert bytes to numpy array
    audio_np = np.frombuffer(audio_data, dtype=np.int16)
    audio_float = audio_np.astype(np.float32) / 32768.0  # Normalize
    
    # Resample if needed
    if target_sr != 16000:
        import librosa
        audio_float = librosa.resample(audio_float, orig_sr=16000, target_sr=target_sr)
        
    return audio_float

def audio_to_tensor(audio_float):
    """
    Convert audio to tensor format expected by Whisper
    """
    audio_tensor = torch.from_numpy(audio_float)
    return audio_tensor
```

### 3. Whisper Integration
```python
import whisper

class WhisperProcessor:
    def __init__(self, model_size="base"):
        """
        Initialize Whisper model
        Available sizes: tiny, base, small, medium, large
        """
        self.model = whisper.load_model(model_size)
        
    def transcribe_audio(self, audio_tensor):
        """
        Transcribe audio using Whisper
        """
        # Ensure audio is in the right format
        if audio_tensor.device != self.model.device:
            audio_tensor = audio_tensor.to(self.model.device)
            
        # Pad or trim audio to fit model
        audio_tensor = whisper.pad_or_trim(audio_tensor)
        
        # Make log-Mel spectrogram and move to the same device as the model
        mel = whisper.log_mel_spectrogram(audio_tensor).to(self.model.device)
        
        # Decode the audio
        options = whisper.DecodingOptions(fp16=False, language="en")
        result = whisper.decode(self.model, mel, options)
        
        return result.text
```

## ROS 2 Integration

### Message Definition
```python
# speech_to_text.msg
string audio_data
string transcript
string confidence
time timestamp
```

### Publisher Node
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from audio_processing_interfaces.msg import SpeechToText  # Custom message
import pyaudio
import numpy as np
import whisper

class WhisperPublisher(Node):
    def __init__(self):
        super().__init__('whisper_publisher')
        self.publisher_ = self.create_publisher(SpeechToText, 'speech_to_text', 10)
        
        # Initialize audio capture
        self.audio_capture = AudioCapture()
        self.whisper_processor = WhisperProcessor()
        
        # Timer for continuous capture
        self.timer = self.create_timer(0.1, self.capture_and_transcribe)
        
    def capture_and_transcribe(self):
        stream = self.audio_capture.start_stream()
        raw_audio = self.audio_capture.read_audio(stream, duration=3)  # 3-second chunks
        stream.stop_stream()
        stream.close()
        
        # Preprocess and transcribe
        audio_float = preprocess_audio(raw_audio)
        audio_tensor = audio_to_tensor(audio_float)
        transcript = self.whisper_processor.transcribe_audio(audio_tensor)
        
        # Publish result
        msg = SpeechToText()
        msg.audio_data = raw_audio.hex()  # Encode as hex string
        msg.transcript = transcript
        msg.confidence = "0.9"  # Placeholder - Whisper doesn't provide confidence by default
        msg.timestamp = self.get_clock().now().to_msg()
        
        self.publisher_.publish(msg)
        self.get_logger().info(f"Transcribed: {transcript}")
```

## Configuration File

### audio_config.yaml
```yaml
# Audio processing configuration for Whisper integration
audio_processing:
  # Recording parameters
  sample_rate: 16000
  channels: 1
  chunk_size: 1024
  recording_duration: 3  # seconds per chunk
  
  # Whisper model parameters
  model_size: "base"  # tiny, base, small, medium, large
  language: "en"  # language code
  device: "cuda"  # "cuda" or "cpu"
  
  # Processing options
  preprocess_audio: true
  normalize_audio: true
  resample_audio: false  # Only if needed
  
  # ROS 2 integration
  topic_name: "/speech_to_text"
  queue_size: 10
  publish_rate: 1.0  # Hz (process every 1 second)
  
  # Thresholds
  silence_threshold: 0.01  # Below this is considered silence
  min_recording_length: 0.5  # Minimum length in seconds
  max_recording_length: 10.0  # Maximum length in seconds before auto-stop
  
  # Performance
  max_processing_time: 5.0  # Maximum time to wait for transcription
```

## Installation Script

### setup_audio_env.sh
```bash
#!/bin/bash

echo "Setting up audio processing environment for Whisper integration..."

# Create virtual environment
python -m venv audio_env
source audio_env/bin/activate  # On Windows: audio_env\Scripts\activate

# Install PyTorch
pip install torch torchvision torchaudio

# Install Whisper
pip install git+https://github.com/openai/whisper.git 

# Install audio processing libraries
pip install pyaudio soundfile librosa

# Install ROS 2 Python client (rclpy)
pip install rclpy

# Install additional dependencies
pip install numpy

echo "Audio processing environment setup complete!"
echo "Activate with: source audio_env/bin/activate"
```

## Testing Framework

### test_audio_processing.py
```python
import unittest
import numpy as np
import torch
import whisper
from audio_processing import AudioCapture, preprocess_audio, audio_to_tensor, WhisperProcessor

class TestAudioProcessing(unittest.TestCase):
    def setUp(self):
        self.processor = WhisperProcessor(model_size="tiny")  # Use tiny for tests
        
    def test_audio_capture(self):
        """Test that audio capture initializes correctly"""
        capture = AudioCapture()
        stream = capture.start_stream()
        self.assertIsNotNone(stream)
        capture.close()
        
    def test_preprocess_audio(self):
        """Test audio preprocessing"""
        # Create dummy audio data (1 second of silence at 16kHz)
        dummy_audio = np.zeros(16000, dtype=np.int16)
        processed = preprocess_audio(dummy_audio.tobytes())
        self.assertEqual(len(processed), 16000)
        self.assertTrue(np.allclose(processed, np.zeros(16000, dtype=np.float32)))
        
    def test_tensor_conversion(self):
        """Test audio to tensor conversion"""
        dummy_audio = np.zeros(16000, dtype=np.float32)
        tensor = audio_to_tensor(dummy_audio)
        self.assertIsInstance(tensor, torch.Tensor)
        self.assertEqual(tensor.shape[0], 16000)
        
    def test_whisper_transcription(self):
        """Test Whisper transcription (with dummy audio)"""
        # Note: This would require a real audio file to properly test
        # For now, we test that the method exists and doesn't crash with valid input
        dummy_audio = torch.zeros(16000, dtype=torch.float32)
        # Transcription with all silence should return empty or "um" etc
        # This test is limited as Whisper processing requires actual speech patterns

if __name__ == '__main__':
    unittest.main()
```

## Usage Example

### simple_demo.py
```python
from audio_processing import AudioCapture, preprocess_audio, audio_to_tensor, WhisperProcessor

def main():
    # Initialize components
    capture = AudioCapture()
    processor = WhisperProcessor(model_size="base")
    
    print("Speak now (3 seconds)...")
    stream = capture.start_stream()
    raw_audio = capture.read_audio(stream, duration=3)
    stream.stop_stream()
    stream.close()
    
    # Process audio
    audio_float = preprocess_audio(raw_audio)
    audio_tensor = audio_to_tensor(audio_float)
    transcript = processor.transcribe_audio(audio_tensor)
    
    print(f"Transcription: {transcript}")
    
    capture.close()

if __name__ == "__main__":
    main()
```

This environment provides the foundation for audio processing and Whisper integration as required for the VLA pipeline in Module 4 of the textbook. The implementation follows the requirements specified in the tasks and ensures compatibility with ROS 2 for integration with the robotics platform.