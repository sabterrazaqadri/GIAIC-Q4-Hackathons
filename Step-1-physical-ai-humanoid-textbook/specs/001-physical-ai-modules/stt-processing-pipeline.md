# Speech-to-Text Processing Pipeline for Humanoid Robotics

## Overview

This document outlines the complete speech-to-text processing pipeline specifically designed for humanoid robotics applications. The pipeline encompasses audio capture, preprocessing, speech recognition (using Whisper), natural language understanding, and integration with robot action systems. This pipeline is essential for enabling voice-command-driven robot actions as required in Module 4 of the Physical AI & Humanoid Robotics textbook.

## Architecture Overview

### High-Level Pipeline Flow

```
Audio Input → Preprocessing → Speech Recognition → Language Understanding → Action Planning → Safety Validation → Robot Execution
     ↓              ↓                ↓                   ↓                    ↓              ↓              ↓
 Microphone    Denoising      Whisper STT      Intent Recognition    LLM Planning    Validation     Actions
 Array/Sensor  Enhancement    Transcription    & Slot Filling        & Reasoning     Layers        Execution
```

## 1. Audio Input and Capture Module

### 1.1. Audio Sensor Configuration

For humanoid robotics applications, the audio capture system must be carefully configured to handle the unique challenges of mobile robotic platforms:

```yaml
# Audio input configuration for humanoid robot
audio_input:
  microphone_array:
    # Configuration for microphone array on humanoid robot
    count: 4  # Number of microphones for beamforming
    positions:  # 3D positions relative to robot head (x, y, z in meters)
      - [0.05, 0.0, 0.1]   # Front
      - [-0.05, 0.0, 0.1]  # Rear  
      - [0.0, 0.05, 0.1]   # Left
      - [0.0, -0.05, 0.1]  # Right
    sample_rate: 16000      # Standard for speech recognition
    bit_depth: 16           # Bit depth for audio samples
    channels: 1             # Mono output after beamforming
    
  beamforming:
    # Parameters for acoustic beamforming
    enabled: true
    target_direction: [0, 0, 1]  # Pointing forward (robot's forward direction)
    beam_width: 60               # Angular width of sensitivity beam in degrees
    null_steering_enabled: true  # Suppress sounds from other directions
    null_angles: [90, 270]       # Angles where sensitivity is minimized (left/right)
    
  noise_reduction:
    # Acoustic noise reduction parameters
    enabled: true
    algorithm: "spectral_subtraction"  # Options: spectral_subtraction, wiener_filter, mlmf
    noise_threshold_db: -30           # Threshold for considering audio as noise
    suppression_level_db: -15         # Amount of noise suppression in dB
    
  preprocessing:
    # General audio preprocessing
    high_pass_filter_freq: 80     # Remove low-frequency rumble
    gain_control: 
      enabled: true
      target_level_dbfs: -20      # Target signal level in dBFS
      level_range_db: 10          # Range adjustment allowed in dB
```

### 1.2. Real-Time Audio Processing

The audio capture module implements real-time processing for continuous monitoring:

```python
import pyaudio
import numpy as np
import threading
import queue
import webrtcvad
from collections import deque
import time

class AudioCapture:
    def __init__(self, config):
        self.config = config
        self.audio = pyaudio.PyAudio()
        self.running = False
        self.audio_buffer = deque(maxlen=30)  # Store up to 30 seconds of audio
        self.vad = webrtcvad.Vad(2)  # Aggressiveness level (0-3, 2 is default)
        
        # Initialize the audio stream
        self.stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=config['channels'],
            rate=config['sample_rate'],
            input=True,
            frames_per_buffer=config['chunk_size'],
            stream_callback=self._audio_callback
        )
        
    def _audio_callback(self, in_data, frame_count, time_info, status):
        """
        Callback function for real-time audio capture
        """
        # Convert to numpy array
        audio_array = np.frombuffer(in_data, dtype=np.int16).astype(np.float32) / 32768.0
        
        # Apply preprocessing
        processed_audio = self._preprocess_audio(audio_array)
        
        # Add to buffer for downstream processing
        if self.running:
            self.audio_buffer.append(processed_audio)
        
        return (in_data, pyaudio.paContinue)
    
    def _preprocess_audio(self, audio_data):
        """
        Apply preprocessing to audio data
        """
        # Apply gain control
        if self.config['gain_control']['enabled']:
            audio_data = self._apply_gain_control(audio_data)
        
        # Apply high-pass filter
        if self.config['high_pass_filter_freq'] > 0:
            audio_data = self._apply_high_pass_filter(audio_data)
        
        return audio_data
    
    def start_capture(self):
        """
        Start audio capture
        """
        self.running = True
        self.stream.start_stream()
    
    def stop_capture(self):
        """
        Stop audio capture
        """
        self.running = False
        self.stream.stop_stream()
    
    def get_recent_audio(self, duration_seconds=1.0):
        """
        Get recent audio from buffer
        """
        samples_needed = int(self.config['sample_rate'] * duration_seconds)
        recent_audio = []
        
        current_samples = 0
        for i in range(len(self.audio_buffer)-1, -1, -1):
            chunk = self.audio_buffer[i]
            recent_audio.insert(0, chunk)
            current_samples += len(chunk)
            
            if current_samples >= samples_needed:
                break
        
        if recent_audio:
            combined_audio = np.concatenate(recent_audio)
            # Ensure we return exactly the requested duration
            if len(combined_audio) > samples_needed:
                return combined_audio[-samples_needed:]
            else:
                return combined_audio
        else:
            return np.array([])
```

## 2. Audio Preprocessing Module

### 2.1. Noise Reduction and Enhancement

The preprocessing module handles cleaning of audio signals for improved recognition accuracy:

```python
import scipy.signal as signal
import librosa
import soundfile as sf

class AudioPreprocessor:
    def __init__(self, config):
        self.config = config
        self.sample_rate = config['sample_rate']
        self.high_pass_freq = config['high_pass_filter_freq']
        
        # Initialize high-pass filter coefficients
        nyquist = self.sample_rate / 2
        normalized_freq = self.high_pass_freq / nyquist
        self.b, self.a = signal.butter(4, normalized_freq, btype='high', analog=False)
    
    def preprocess_audio(self, audio_data):
        """
        Apply preprocessing to audio data
        """
        # Apply high-pass filter to remove low-frequency noise
        if self.high_pass_freq > 0:
            audio_data = signal.filtfilt(self.b, self.a, audio_data)
        
        # Normalize audio levels
        audio_data = self._normalize_audio(audio_data)
        
        # Apply noise reduction if enabled
        if self.config['noise_reduction']['enabled']:
            audio_data = self._apply_noise_reduction(audio_data)
        
        # Apply gain control
        if self.config['gain_control']['enabled']:
            target_db = self.config['gain_control']['target_level_dbfs']
            audio_data = self._apply_compressor(audio_data, target_db)
        
        return audio_data
    
    def _normalize_audio(self, audio_data):
        """
        Normalize audio to prevent clipping and adjust levels
        """
        # Calculate peak amplitude
        max_amplitude = np.max(np.abs(audio_data))
        
        # Normalize to prevent clipping
        if max_amplitude > 1.0:
            audio_data = audio_data / max_amplitude
        
        # Adjust to target level if specified
        target_level = 10**(self.config['gain_control']['target_level_dbfs']/20.0)
        current_rms = np.sqrt(np.mean(audio_data**2))
        
        if current_rms > 0:
            target_gain = target_level / current_rms
            # Limit gain to avoid excessive amplification of noise
            gain_limit = 10**(self.config['gain_control']['level_range_db']/20.0)
            target_gain = min(target_gain, gain_limit)
            audio_data = audio_data * target_gain
        
        return np.clip(audio_data, -1.0, 1.0)
    
    def _apply_noise_reduction(self, audio_data):
        """
        Apply basic noise reduction using spectral subtraction
        """
        # Convert to frequency domain using STFT
        stft = librosa.stft(audio_data)
        magnitude = np.abs(stft)
        phase = np.angle(stft)
        
        # Estimate noise (using minimum statistics or simple averaging)
        noise_floor = np.percentile(magnitude, 10, axis=1, keepdims=True)  # 10th percentile as noise estimate
        
        # Subtract noise (with flooring to avoid over-subtraction)
        enhanced_magnitude = np.maximum(magnitude - noise_floor * 0.8, magnitude * 0.2)
        
        # Reconstruct the signal
        enhanced_stft = enhanced_magnitude * np.exp(1j * phase)
        enhanced_audio = librosa.istft(enhanced_stft)
        
        return enhanced_audio
    
    def _apply_compressor(self, audio_data, target_level_db):
        """
        Apply dynamic range compression
        """
        # Calculate RMS level
        rms = np.sqrt(np.mean(audio_data**2))
        current_level_db = 20 * np.log10(max(rms, 1e-10))
        
        # Calculate required gain
        required_gain_db = target_level_db - current_level_db
        required_gain = 10**(required_gain_db / 20.0)
        
        return audio_data * required_gain
```

### 2.2. Voice Activity Detection (VAD)

Detecting speech segments from continuous audio:

```python
class VoiceActivityDetector:
    def __init__(self, config):
        self.config = config
        self.vad = webrtcvad.Vad(config.get('vad_aggressiveness', 2))
        self.sample_rate = config['sample_rate']
        self.frame_duration_ms = 30  # WebRTC VAD requires 10, 20, or 30 ms frames
        self.frame_size = self.sample_rate * self.frame_duration_ms // 1000
        
    def is_speech_detected(self, audio_data, threshold_ratio=0.5):
        """
        Detect if speech is present in the audio segment
        Returns True if speech is detected, False otherwise
        """
        # Ensure audio length is divisible by frame size
        remainder = len(audio_data) % self.frame_size
        if remainder != 0:
            # Pad with zeros if necessary
            audio_data = np.pad(audio_data, (0, self.frame_size - remainder), mode='constant')
        
        # Split into frames
        frames = []
        for i in range(0, len(audio_data), self.frame_size):
            frame = audio_data[i:i+self.frame_size]
            if len(frame) == self.frame_size:
                # Convert to 16-bit integer for WebRTC VAD
                frame_int16 = (frame * 32767).astype(np.int16)
                frames.append(frame_int16.tobytes())
        
        # Run VAD on frames
        speech_frames = 0
        total_frames = len(frames)
        
        for frame in frames:
            try:
                if self.vad.is_speech(frame, self.sample_rate):
                    speech_frames += 1
            except Exception:
                # Handle errors in VAD processing
                continue
        
        # Return True if ratio of speech frames exceeds threshold
        return speech_frames / max(total_frames, 1) > threshold_ratio if total_frames > 0 else False
    
    def detect_speech_segments(self, audio_data, min_segment_duration=0.5):
        """
        Detect speech segments in longer audio clip
        Returns list of (start_time, end_time) tuples for speech segments
        """
        segments = []
        frame_duration = self.frame_size / self.sample_rate
        
        # Process in frames
        current_speech_start = None
        for i in range(0, len(audio_data), self.frame_size):
            frame = audio_data[i:i+self.frame_size]
            
            if len(frame) < self.frame_size:
                break  # Last frame is too short
                
            # Convert to bytes for VAD
            frame_int16 = (frame * 32767).astype(np.int16)
            frame_bytes = frame_int16.tobytes()
            
            try:
                is_speech = self.vad.is_speech(frame_bytes, self.sample_rate)
                current_time = i / self.sample_rate
                
                if is_speech and current_speech_start is None:
                    # Start of speech segment
                    current_speech_start = current_time
                elif not is_speech and current_speech_start is not None:
                    # End of speech segment
                    duration = current_time - current_speech_start
                    if duration >= min_segment_duration:
                        segments.append((current_speech_start, current_time))
                    current_speech_start = None
            except Exception:
                continue
        
        # Handle case where speech segment continues to end of audio
        if current_speech_start is not None:
            end_time = len(audio_data) / self.sample_rate
            duration = end_time - current_speech_start
            if duration >= min_segment_duration:
                segments.append((current_speech_start, end_time))
        
        return segments
```

## 3. Speech Recognition Module (Whisper Integration)

### 3.1. Whisper Integration Layer

The core speech recognition component using OpenAI Whisper:

```python
import whisper
import torch
import numpy as np
from typing import Dict, Any, Optional

class WhisperSTT:
    def __init__(self, model_size="base", device=None):
        """
        Initialize Whisper Speech-to-Text engine
        
        Args:
            model_size: Size of Whisper model ('tiny', 'base', 'small', 'medium', 'large')
            device: Device for processing ('cuda', 'cpu', or None for auto)
        """
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model = whisper.load_model(model_size, device=self.device)
        self.model_size = model_size
        
        # Initialize audio processing parameters
        self.sample_rate = 16000  # Whisper standard
        self.max_audio_duration = 30  # Maximum single processing duration in seconds
        
    def transcribe_audio(self, audio_data: np.ndarray, 
                        language: Optional[str] = "en",
                        temperature: float = 0.0) -> Dict[str, Any]:
        """
        Transcribe audio data using Whisper
        
        Args:
            audio_data: Audio signal as numpy array (normalized to [-1, 1])
            language: Language code (e.g., 'en', 'es', 'fr')
            temperature: Temperature for decoding (0.0 for deterministic)
        
        Returns:
            Dictionary with transcription results
        """
        # Ensure audio is in correct format
        if len(audio_data) == 0:
            return {
                "text": "",
                "confidence": 0.0,
                "segments": [],
                "language": language or "unknown"
            }
        
        # Pad or trim audio to appropriate length
        target_samples = self.max_audio_duration * self.sample_rate
        if len(audio_data) > target_samples:
            # For longer audio, we might need to process in chunks
            return self._transcribe_long_audio(audio_data, language, temperature)
        
        # Pad if needed
        audio_data = whisper.pad_or_trim(audio_data, length=target_samples)
        
        # Convert to tensor and move to appropriate device
        mel = whisper.log_mel_spectrogram(audio_data).to(self.model.device)
        
        # Decode with specified language if provided
        options = whisper.DecodingOptions(
            language=language,
            temperature=temperature,
            without_timestamps=True,
            fp16=torch.float16 == self.model.dtype
        )
        
        result = whisper.decode(self.model, mel, options)
        
        return {
            "text": result.text.strip(),
            "confidence": getattr(result, 'avg_logprob', 0.0),  # Confidence metric
            "segments": getattr(result, 'segments', []),
            "language": getattr(result, 'language', language or "unknown"),
            "processing_time": getattr(result, 'processing_time', 0.0)
        }
    
    def _transcribe_long_audio(self, audio_data: np.ndarray, 
                              language: Optional[str], 
                              temperature: float) -> Dict[str, Any]:
        """
        Process audio longer than Whisper's maximum input duration
        This method splits long audio into chunks and processes them
        """
        chunk_duration = self.max_audio_duration - 2  # Leave 2s buffer
        chunk_samples = int(chunk_duration * self.sample_rate)
        total_samples = len(audio_data)
        
        full_text = ""
        total_confidence = 0.0
        processed_chunks = 0
        
        for start_idx in range(0, total_samples, chunk_samples):
            end_idx = min(start_idx + chunk_samples, total_samples)
            chunk = audio_data[start_idx:end_idx]
            
            # Process the chunk
            chunk_result = self._transcribe_chunk(chunk, language, temperature)
            
            if chunk_result["text"].strip():
                if full_text:
                    full_text += " " + chunk_result["text"]
                else:
                    full_text = chunk_result["text"]
                
                total_confidence += chunk_result["confidence"]
                processed_chunks += 1
        
        avg_confidence = total_confidence / max(processed_chunks, 1) if processed_chunks > 0 else 0.0
        
        return {
            "text": full_text,
            "confidence": avg_confidence,
            "segments": [],  # For brevity, not returning all segments for long audio
            "language": language or "unknown",
            "processed_chunks": processed_chunks
        }
    
    def _transcribe_chunk(self, audio_chunk: np.ndarray, 
                         language: Optional[str], 
                         temperature: float) -> Dict[str, Any]:
        """
        Transcribe a single chunk of audio
        """
        # Pad or trim the chunk to standard length
        padded_chunk = whisper.pad_or_trim(audio_chunk, 
                                          length=self.max_audio_duration * self.sample_rate)
        
        # Convert to mel spectrogram
        mel = whisper.log_mel_spectrogram(padded_chunk).to(self.model.device)
        
        # Decode
        options = whisper.DecodingOptions(
            language=language,
            temperature=temperature,
            without_timestamps=True,
            fp16=torch.float16 == self.model.dtype
        )
        
        result = whisper.decode(self.model, mel, options)
        
        return {
            "text": result.text.strip(),
            "confidence": getattr(result, 'avg_logprob', 0.0)
        }

class StreamingWhisperSTT:
    """
    Streaming STT using Whisper for real-time applications
    """
    def __init__(self, model_size="base", device=None, 
                 min_segment_duration=1.0, confidence_threshold=0.5):
        self.whisper_stt = WhisperSTT(model_size, device)
        self.min_segment_duration = min_segment_duration
        self.confidence_threshold = confidence_threshold
        
        # Buffer for accumulating audio from real-time capture
        self.audio_buffer = np.array([])
        self.buffer_max_duration = 30  # Maximum buffer duration
        self.buffer_max_samples = int(self.buffer_max_duration * 16000)
    
    def add_audio_chunk(self, audio_chunk: np.ndarray) -> Optional[Dict[str, Any]]:
        """
        Add an audio chunk to the buffer and check if we should process it
        Returns transcription result if buffer contains speech segment, None otherwise
        """
        # Append new chunk to buffer
        self.audio_buffer = np.concatenate([self.audio_buffer, audio_chunk])
        
        # Limit buffer size to prevent excessive memory usage
        if len(self.audio_buffer) > self.buffer_max_samples:
            # Keep only the most recent audio samples
            self.audio_buffer = self.audio_buffer[-self.buffer_max_samples:]
        
        # Check if we have enough audio to process
        if len(self.audio_buffer) >= self.min_segment_duration * 16000:
            # Process the accumulated audio
            result = self.whisper_stt.transcribe_audio(
                audio_data=self.audio_buffer,
                language="en"
            )
            
            # Only return result if confidence is above threshold
            if result["confidence"] > self.confidence_threshold:
                # Clear the buffer after processing
                transcript_len = self._estimate_audio_length(result["text"])
                samples_to_keep = max(0, len(self.audio_buffer) - int(transcript_len * 16000))
                self.audio_buffer = self.audio_buffer[-samples_to_keep:] if samples_to_keep > 0 else np.array([])
                
                return result
            else:
                # Keep most recent audio, remove processed portion estimate
                # (since we don't know exact alignment, keep only 50% of buffer)
                half_buffer = len(self.audio_buffer) // 2
                self.audio_buffer = self.audio_buffer[half_buffer:]
        
        return None
    
    def _estimate_audio_length(self, text: str) -> float:
        """
        Estimate audio length based on text length (very rough approximation)
        Averages around 4-5 words per second in normal speech
        """
        word_count = len(text.split())
        return max(word_count / 4.0, 1.0)  # Minimum 1 second
```

## 4. Natural Language Understanding (NLU) Module

### 4.1. Intent Recognition and Entity Extraction

Processing the Whisper output to extract robot commands:

```python
import re
from enum import Enum
from dataclasses import dataclass
from typing import List, Dict, Optional, Any

class RobotIntent(Enum):
    NAVIGATE_TO = "navigate_to"
    GRASP_OBJECT = "grasp_object"
    FOLLOW_PERSON = "follow_person"
    STOP_MOVEMENT = "stop_movement"
    SPEAK_RESPONSE = "speak_response"
    LOOK_AT = "look_at"
    GO_HOME = "go_home"
    CHAT_CONVERSATION = "chat_conversation"
    UNDEFINED = "undefined"

@dataclass
class CommandEntity:
    entity_type: str
    value: str
    confidence: float
    start_char: int
    end_char: int

@dataclass
class ParsedCommand:
    intent: RobotIntent
    entities: List[CommandEntity]
    original_text: str
    confidence: float
    extra_data: Dict[str, Any]

class NLUProcessor:
    def __init__(self):
        # Define patterns for recognizing intents and entities
        self.intent_patterns = {
            RobotIntent.NAVIGATE_TO: [
                r"go to (?P<location>\w+(?:\s+\w+)*)",
                r"navigate to (?P<location>\w+(?:\s+\w+)*)",
                r"move to (?P<location>\w+(?:\s+\w+)*)",
                r"reach (?P<location>\w+(?:\s+\w+)*)",
                r"walk to (?P<location>\w+(?:\s+\w+)*)",
                r"drive to (?P<location>\w+(?:\s+\w+)*)"
            ],
            RobotIntent.GRASP_OBJECT: [
                r"pick up (?P<object>\w+(?:\s+\w+)*)",
                r"grasp (?P<object>\w+(?:\s+\w+)*)",
                r"take (?P<object>\w+(?:\s+\w+)*)",
                r"hold (?P<object>\w+(?:\s+\w+)*)",
                r"get (?P<object>\w+(?:\s+\w+)*)",
                r"catch (?P<object>\w+(?:\s+\w+)*)"
            ],
            RobotIntent.FOLLOW_PERSON: [
                r"follow (?P<person>\w+(?:\s+\w+)*)",
                r"track (?P<person>\w+(?:\s+\w+)*)",
                r"go after (?P<person>\w+(?:\s+\w+)*)",
                r"chase (?P<person>\w+(?:\s+\w+)*)"
            ],
            RobotIntent.STOP_MOVEMENT: [
                r"stop",
                r"halt",
                r"pause",
                r"freeze",
                r"stand still"
            ],
            RobotIntent.SPEAK_RESPONSE: [
                r"say (?P<response>.+)",
                r"speak (?P<response>.+)",
                r"tell (?P<response>.+)",
                r"repeat (?P<response>.+)"
            ],
            RobotIntent.LOOK_AT: [
                r"look at (?P<object>\w+(?:\s+\w+)*)",
                r"face (?P<object>\w+(?:\s+\w+)*)",
                r"turn to (?P<object>\w+(?:\s+\w+)*)",
                r"stare at (?P<object>\w+(?:\s+\w+)*)"
            ],
            RobotIntent.GO_HOME: [
                r"go home",
                r"return home",
                r"return to base",
                r"go to charging station"
            ]
        }
        
        # Define entity extraction patterns
        self.entity_patterns = {
            'location': [
                r"kitchen|living room|bedroom|office|bathroom|dining room|hallway|entrance|exit|door|window"
            ],
            'object': [
                r"ball|mug|cup|book|phone|tablet|keys|wallet|glass|plate|fork|knife|spoon|chair|table"
            ],
            'person': [
                r"me|you|him|her|them|person|man|woman|child|adult|elderly|grandma|grandpa|friend|stranger"
            ]
        }
    
    def parse_intent(self, text: str) -> ParsedCommand:
        """
        Parse the intent and entities from the transcribed text
        """
        text_lower = text.lower().strip()
        intent = RobotIntent.UNDEFINED
        entities = []
        confidence = 0.0
        matched_pattern = None
        
        # Try to match each intent pattern
        for intent_type, patterns in self.intent_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower, re.IGNORECASE)
                if match:
                    intent = intent_type
                    matched_pattern = pattern
                    
                    # Extract entities from the match
                    for entity_name, entity_value in match.groupdict().items():
                        entity_confidence = self._calculate_entity_confidence(entity_name, entity_value)
                        entities.append(CommandEntity(
                            entity_type=entity_name,
                            value=entity_value,
                            confidence=entity_confidence,
                            start_char=match.start(entity_name),
                            end_char=match.end(entity_name)
                        ))
                    
                    # Calculate confidence based on pattern match quality
                    confidence = self._calculate_intent_confidence(intent_type, text_lower, match)
                    break
            
            if intent != RobotIntent.UNDEFINED:
                break
        
        # If no pattern matched, use more general classification
        if intent == RobotIntent.UNDEFINED:
            intent, confidence = self._classify_undefined_intent(text_lower)
        
        return ParsedCommand(
            intent=intent,
            entities=entities,
            original_text=text,
            confidence=confidence,
            extra_data={"matched_pattern": matched_pattern}
        )
    
    def _calculate_intent_confidence(self, intent: RobotIntent, text: str, match) -> float:
        """
        Calculate confidence score for detected intent
        """
        # Base confidence on how well the pattern matched
        confidence = 0.8  # Base confidence
        
        # Increase confidence for longer matches
        match_ratio = len(match.group()) / len(text)
        confidence += min(0.2, match_ratio * 0.1)
        
        # Consider text length - shorter, more direct commands usually have higher confidence
        if len(text) < 20:
            confidence += 0.1
        
        return min(confidence, 1.0)
    
    def _calculate_entity_confidence(self, entity_type: str, entity_value: str) -> float:
        """
        Calculate confidence for extracted entity
        """
        confidence = 0.7  # Base confidence
        
        # Check against predefined entity patterns
        if entity_type in self.entity_patterns:
            for pattern in self.entity_patterns[entity_type]:
                if re.search(pattern, entity_value, re.IGNORECASE):
                    confidence = 0.9
                    break
        
        # Increase confidence for longer entities (more specific)
        if len(entity_value) > 8:
            confidence += 0.1
        
        return min(confidence, 1.0)
    
    def _classify_undefined_intent(self, text: str) -> tuple:
        """
        Classify undefined intents using more general patterns
        """
        # Look for general command patterns that don't fit specific intents
        if any(word in text for word in ["hello", "hi", "hey", "please"]):
            return RobotIntent.CHAT_CONVERSATION, 0.6
        
        # General confidence for unknown commands
        return RobotIntent.UNDEFINED, 0.4
```

### 4.2. Robot Command Generation

Converting parsed intents into robot-executable commands:

```python
from typing import Union
import json

class CommandGenerator:
    def __init__(self, robot_config):
        self.robot_config = robot_config
    
    def generate_robot_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Convert parsed command into robot-executable format
        """
        if parsed_command.intent == RobotIntent.NAVIGATE_TO:
            return self._generate_navigation_command(parsed_command)
        elif parsed_command.intent == RobotIntent.GRASP_OBJECT:
            return self._generate_grasp_command(parsed_command)
        elif parsed_command.intent == RobotIntent.FOLLOW_PERSON:
            return self._generate_follow_command(parsed_command)
        elif parsed_command.intent == RobotIntent.STOP_MOVEMENT:
            return self._generate_stop_command(parsed_command)
        elif parsed_command.intent == RobotIntent.SPEAK_RESPONSE:
            return self._generate_speak_command(parsed_command)
        elif parsed_command.intent == RobotIntent.LOOK_AT:
            return self._generate_lookat_command(parsed_command)
        elif parsed_command.intent == RobotIntent.GO_HOME:
            return self._generate_gohome_command(parsed_command)
        else:
            return self._generate_fallback_command(parsed_command)
    
    def _generate_navigation_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate navigation command for the robot
        """
        location = self._resolve_location(parsed_command.entities)
        
        return {
            "command_type": "navigation",
            "action": "navigate_to_pose",
            "parameters": {
                "target_location": location,
                "target_coordinates": self._get_coordinates_for_location(location)
            },
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time()
            }
        }
    
    def _generate_grasp_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate grasping command for the robot
        """
        obj = self._find_object_in_environment(parsed_command.entities)
        
        return {
            "command_type": "manipulation",
            "action": "grasp_object",
            "parameters": {
                "target_object": obj,
                "object_position": self._get_object_position(obj) if obj else None
            },
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time()
            }
        }
    
    def _generate_follow_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate person-following command for the robot
        """
        person = self._identify_person(parsed_command.entities)
        
        return {
            "command_type": "navigation",
            "action": "follow_target",
            "parameters": {
                "target_type": "person",
                "target_identifier": person,
                "following_distance": 1.5  # meters
            },
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time()
            }
        }
    
    def _generate_stop_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate stop command for the robot
        """
        return {
            "command_type": "motion",
            "action": "stop",
            "parameters": {},
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time(),
                "urgency": "high"
            }
        }
    
    def _generate_speak_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate speech command for the robot
        """
        response = None
        for entity in parsed_command.entities:
            if entity.entity_type == "response":
                response = entity.value
                break
        
        return {
            "command_type": "interaction",
            "action": "speak",
            "parameters": {
                "text": response or parsed_command.original_text.replace("say ", "").replace("speak ", ""),
                "voice_pitch": 1.0,
                "speaking_rate": 1.0
            },
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time()
            }
        }
    
    def _generate_lookat_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate look-at command for the robot
        """
        target = self._find_target_object(parsed_command.entities)
        
        return {
            "command_type": "motion",
            "action": "look_at",
            "parameters": {
                "target_object": target,
                "target_position": self._get_object_position(target) if target else None
            },
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time()
            }
        }
    
    def _generate_gohome_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate return-home command for the robot
        """
        return {
            "command_type": "navigation",
            "action": "navigate_to_pose",
            "parameters": {
                "target_location": "home",
                "target_coordinates": self.robot_config.home_position
            },
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time()
            }
        }
    
    def _generate_fallback_command(self, parsed_command: ParsedCommand) -> Dict[str, Any]:
        """
        Generate fallback command when intent is unclear
        """
        return {
            "command_type": "fallback",
            "action": "request_clarification",
            "parameters": {
                "recognized_text": parsed_command.original_text,
                "confidence": parsed_command.confidence
            },
            "metadata": {
                "original_text": parsed_command.original_text,
                "confidence": parsed_command.confidence,
                "timestamp": time.time()
            }
        }
    
    def _resolve_location(self, entities: List[CommandEntity]) -> str:
        """
        Resolve location entities to known locations in the environment
        """
        for entity in entities:
            if entity.entity_type == "location":
                return entity.value
        return "unknown"
    
    def _find_object_in_environment(self, entities: List[CommandEntity]) -> Optional[str]:
        """
        Find the specified object in the robot's environment
        """
        for entity in entities:
            if entity.entity_type == "object":
                # In a real implementation, this would query the robot's perception system
                return entity.value
        return None
    
    def _identify_person(self, entities: List[CommandEntity]) -> Optional[str]:
        """
        Identify the person to follow
        """
        for entity in entities:
            if entity.entity_type == "person":
                return entity.value
        return "closest_person"
    
    def _find_target_object(self, entities: List[CommandEntity]) -> Optional[str]:
        """
        Find the target object to look at
        """
        for entity in entities:
            if entity.entity_type == "object":
                return entity.value
        return None
    
    def _get_coordinates_for_location(self, location: str) -> Optional[Dict[str, float]]:
        """
        Get coordinates for a named location
        """
        # In a real implementation, this would query a semantic map
        known_locations = {
            "kitchen": {"x": 2.5, "y": -1.0, "z": 0.0},
            "living room": {"x": 0.0, "y": 0.0, "z": 0.0},
            "bedroom": {"x": -2.0, "y": 1.5, "z": 0.0},
            "office": {"x": 1.0, "y": 2.0, "z": 0.0},
            "home": {"x": 0.0, "y": 0.0, "z": 0.0}  # Robot's home/base position
        }
        
        return known_locations.get(location.lower())
    
    def _get_object_position(self, obj_name: str) -> Optional[Dict[str, float]]:
        """
        Get the position of an object in the environment
        """
        # In a real implementation, this would query perception system
        return {"x": 0.0, "y": 0.0, "z": 0.0}
```

## 5. Safety and Validation Module

### 5.1. Command Validation and Safety Checks

Ensuring that voice commands are safe and appropriate for the robot to execute:

```python
from typing import Dict, Any, List

class SafetyValidator:
    def __init__(self, robot_config, environment_map):
        self.robot_config = robot_config
        self.environment_map = environment_map
        self.forbidden_commands = [
            "self destruct", "shutdown", "destroy", 
            "damage", "break", "hurt", "harm", "explode"
        ]
        self.privacy_sensitive_commands = [
            "record", "log", "monitor", "spy", "listen"
        ]
    
    def validate_command(self, robot_command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate whether a robot command is safe and appropriate to execute
        """
        validation_result = {
            "is_valid": True,
            "is_safe": True,
            "is_privacy_safe": True,
            "issues": [],
            "suggestions": []
        }
        
        # Check for forbidden commands
        if robot_command["command_type"] != "fallback":
            original_text_lower = robot_command["metadata"]["original_text"].lower()
            
            for forbidden_cmd in self.forbidden_commands:
                if forbidden_cmd in original_text_lower:
                    validation_result["is_safe"] = False
                    validation_result["issues"].append(f"Forbidden command phrase detected: {forbidden_cmd}")
        
        # Validate navigation commands
        if robot_command["command_type"] == "navigation":
            nav_result = self._validate_navigation_command(robot_command)
            validation_result["is_safe"] = validation_result["is_safe"] and nav_result["is_safe"]
            validation_result["issues"].extend(nav_result["issues"])
            validation_result["suggestions"].extend(nav_result["suggestions"])
        
        # Validate manipulation commands
        elif robot_command["command_type"] == "manipulation":
            manip_result = self._validate_manipulation_command(robot_command)
            validation_result["is_safe"] = validation_result["is_safe"] and manip_result["is_safe"]
            validation_result["issues"].extend(manip_result["issues"])
            validation_result["suggestions"].extend(manip_result["suggestions"])
        
        # Check privacy implications
        for priv_cmd in self.privacy_sensitive_commands:
            if priv_cmd in original_text_lower:
                validation_result["is_privacy_safe"] = False
                validation_result["issues"].append(f"Privacy-sensitive command detected: {priv_cmd}")
        
        # Overall validity check
        validation_result["is_valid"] = (
            validation_result["is_safe"] and 
            validation_result["is_privacy_safe"]
        )
        
        return validation_result
    
    def _validate_navigation_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate navigation-specific safety considerations
        """
        result = {
            "is_safe": True,
            "issues": [],
            "suggestions": []
        }
        
        target_coords = command["parameters"].get("target_coordinates")
        if target_coords:
            # Check if target is in navigable area
            is_navigable = self.environment_map.is_in_navigable_area(target_coords)
            if not is_navigable:
                result["is_safe"] = False
                result["issues"].append(f"Target location {target_coords} is not in navigable area")
                result["suggestions"].append("Please specify a different destination")
            
            # Check for safety around target location
            safety_check = self.environment_map.check_safety_around(target_coords)
            if not safety_check["is_safe"]:
                result["is_safe"] = False
                result["issues"].append(f"Safety issues around target location: {safety_check['details']}")
                result["suggestions"].append("Avoid this area or specify a different destination")
        
        return result
    
    def _validate_manipulation_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate manipulation-specific safety considerations
        """
        result = {
            "is_safe": True,
            "issues": [],
            "suggestions": []
        }
        
        target_obj = command["parameters"].get("target_object")
        if target_obj:
            # Check if object is safe to manipulate
            obj_properties = self.environment_map.get_object_properties(target_obj)
            if obj_properties:
                if obj_properties.get("is_fragile", False):
                    result["is_safe"] = False
                    result["issues"].append(f"Object '{target_obj}' is fragile and may break if grasped")
                    result["suggestions"].append("Be careful when handling this object or choose a different one")
                
                if obj_properties.get("weight", 0) > self.robot_config.max_grasp_weight:
                    result["is_safe"] = False
                    result["issues"].append(f"Object '{target_obj}' is too heavy ({obj_properties['weight']}kg) for robot to grasp")
                    result["suggestions"].append(f"Robot can only grasp objects up to {self.robot_config.max_grasp_weight}kg")
                
                # Check if object is in a safe location to approach and grasp
                obj_position = command["parameters"].get("object_position")
                if obj_position:
                    approachable = self.environment_map.is_approachable_and_graspable(obj_position)
                    if not approachable:
                        result["is_safe"] = False
                        result["issues"].append(f"Object '{target_obj}' is not in a safe location to approach and grasp")
                        result["suggestions"].append("Move to a safe position to approach the object")
        
        return result
```

## 6. Integration with Navigation System (Nav2)

### 6.1. ROS 2 Interface

Connecting the speech processing pipeline to the ROS 2 navigation system:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import threading

class VoiceNavigationInterface(Node):
    def __init__(self):
        super().__init__('voice_navigation_interface')
        
        # Create action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Create publishers and subscribers
        self.speech_sub = self.create_subscription(
            String,
            'speech_commands',
            self.speech_command_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.status_pub = self.create_publisher(
            String,
            'navigation_status',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # Initialize pipeline components
        self.nlu_processor = NLUProcessor()
        self.command_generator = CommandGenerator(robot_config={})
        self.safety_validator = SafetyValidator(
            robot_config={}, 
            environment_map=None  # Would be initialized with actual map
        )
        
        self.get_logger().info('Voice Navigation Interface initialized')
    
    def speech_command_callback(self, msg):
        """
        Process incoming speech commands
        """
        try:
            # Parse the command using NLU
            parsed_command = self.nlu_processor.parse_intent(msg.data)
            
            # Only process confident commands
            if parsed_command.confidence > 0.6:
                # Generate robot command
                robot_command = self.command_generator.generate_robot_command(parsed_command)
                
                # Validate safety
                validation_result = self.safety_validator.validate_command(robot_command)
                
                if validation_result["is_valid"]:
                    # Execute the command based on type
                    if robot_command["command_type"] == "navigation":
                        self._execute_navigation_command(robot_command)
                    elif robot_command["command_type"] == "motion":
                        if robot_command["action"] == "stop":
                            self._execute_stop_command()
                        elif robot_command["action"] == "look_at":
                            self._execute_lookat_command(robot_command)
                else:
                    # Report safety issues
                    self._report_safety_issues(validation_result, robot_command)
            else:
                self.get_logger().info(f'Command confidence too low: {parsed_command.confidence}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing speech command: {e}')
    
    def _execute_navigation_command(self, command):
        """
        Execute navigation command via Nav2
        """
        try:
            # Wait for navigation action server
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Navigation server not available')
                return False
            
            # Create navigation goal
            goal_msg = NavigateToPose.Goal()
            
            # Set target pose
            coords = command["parameters"]["target_coordinates"]
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.pose.pose.position.x = coords["x"]
            goal_msg.pose.pose.position.y = coords["y"]
            goal_msg.pose.pose.position.z = coords["z"]
            
            # For simplicity, assuming zero rotation (in practice, should have proper orientation)
            goal_msg.pose.pose.orientation.w = 1.0
            
            # Send navigation goal
            send_goal_future = self.nav_client.send_goal_async(
                goal_msg,
                feedback_callback=self._navigation_feedback_callback
            )
            
            send_goal_future.add_done_callback(self._navigation_goal_response_callback)
            
            self.get_logger().info(f'Sent navigation goal to: ({coords["x"]}, {coords["y"]})')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error executing navigation command: {e}')
            return False
    
    def _navigation_goal_response_callback(self, future):
        """
        Handle response from navigation server
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._navigation_result_callback)
    
    def _navigation_result_callback(self, future):
        """
        Handle navigation result
        """
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            status_msg = String()
            status_msg.data = 'Navigation to destination completed successfully'
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().info(f'Navigation failed with status: {status}')
            status_msg = String()
            status_msg.data = 'Navigation failed to reach destination'
            self.status_pub.publish(status_msg)
    
    def _navigation_feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        """
        self.get_logger().info(f'Navigation progress: {feedback_msg.feedback.distance_remaining:.2f}m remaining')
    
    def _execute_stop_command(self):
        """
        Execute stop command
        """
        # Cancel any active navigation goals
        cancel_future = self.nav_client._cancel_goal_async()
        self.get_logger().info('Stop command executed - cancelled active goals')
        
        status_msg = String()
        status_msg.data = 'Robot stopped successfully'
        self.status_pub.publish(status_msg)
    
    def _execute_lookat_command(self, command):
        """
        Execute look-at command (would control head/neck joints)
        """
        # In a real implementation, this would control the robot's head/neck joints
        target_obj = command["parameters"]["target_object"]
        self.get_logger().info(f'Look-at command for: {target_obj}')
        
        status_msg = String()
        status_msg.data = f'Looking at {target_obj}'
        self.status_pub.publish(status_msg)
    
    def _report_safety_issues(self, validation_result, command):
        """
        Report safety issues to user
        """
        for issue in validation_result["issues"]:
            self.get_logger().warn(f'Safety issue: {issue}')
        
        status_msg = String()
        status_msg.data = f'Safety issues prevented command execution: {"; ".join(validation_result["issues"])}'
        self.status_pub.publish(status_msg)
```

## 7. Performance Optimization

### 7.1. Resource Management

Managing computational resources efficiently for real-time operation:

```python
import psutil
import GPUtil
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

class ResourceManager:
    def __init__(self, max_cpu_percent=70, max_gpu_memory_percent=80):
        self.max_cpu_percent = max_cpu_percent
        self.max_gpu_memory_percent = max_gpu_memory_percent
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_resources)
        self.monitor_thread.start()
    
    def _monitor_resources(self):
        """
        Monitor system resources in a separate thread
        """
        while self.monitoring:
            cpu_percent = psutil.cpu_percent(interval=1)
            memory_percent = psutil.virtual_memory().percent
            
            # Check if GPU is available and monitor it
            gpus = GPUtil.getGPUs()
            gpu_load = gpus[0].load if gpus else 0
            gpu_memory_percent = gpus[0].memoryUtil if gpus else 0
            
            # Log resource usage
            self.get_logger().debug(
                f'CPU: {cpu_percent}%, Memory: {memory_percent}%, '
                f'GPU: {gpu_load*100:.1f}%, GPU Memory: {gpu_memory_percent*100:.1f}%'
            )
            
            time.sleep(2)  # Monitor every 2 seconds
    
    def execute_with_resource_check(self, function, *args, **kwargs):
        """
        Execute a function with resource availability checking
        """
        # Check current resource usage
        cpu_percent = psutil.cpu_percent(interval=0.1)
        memory_percent = psutil.virtual_memory().percent
        
        # Get GPU usage if available
        gpus = GPUtil.getGPUs()
        gpu_memory_percent = gpus[0].memoryUtil if gpus else 0
        
        if (cpu_percent > self.max_cpu_percent or 
            memory_percent > 85 or  # Memory threshold is different
            gpu_memory_percent > self.max_gpu_memory_percent):
            
            self.get_logger().warn('System resources are high, deferring execution...')
            # Implement backoff strategy or queue for later
            time.sleep(1)  # Brief delay
            return self.execute_with_resource_check(function, *args, **kwargs)
        
        # Execute the function
        return function(*args, **kwargs)
    
    def adaptive_processing(self, audio_data, realtime_factor=1.0):
        """
        Adjust processing based on current system load
        """
        cpu_percent = psutil.cpu_percent(interval=0.1)
        
        if cpu_percent > 80:
            # Reduce model size or processing complexity if CPU is overloaded
            return self._lightweight_process(audio_data)
        elif cpu_percent > 60:
            # Use medium processing for moderate load
            return self._medium_process(audio_data)
        else:
            # Use full processing when resources are available
            return self._full_process(audio_data)
    
    def _lightweight_process(self, audio_data):
        """
        Lightweight processing for high CPU usage scenarios
        """
        # Use smaller model or simpler algorithms
        pass
    
    def _medium_process(self, audio_data):
        """
        Medium processing for moderate CPU usage
        """
        # Use medium-sized model or balanced algorithms
        pass
    
    def _full_process(self, audio_data):
        """
        Full processing for low CPU usage scenarios
        """
        # Use full model and comprehensive algorithms
        pass
    
    def shutdown(self):
        """
        Shutdown resource management
        """
        self.monitoring = False
        self.executor.shutdown(wait=True)
```

## 8. Best Practices and Recommendations

### 8.1. Pipeline Configuration
The speech-to-text processing pipeline should be configured based on specific robot and application requirements:

1. **Model Selection**: Choose Whisper model size based on required accuracy vs. computational efficiency
2. **Threshold Tuning**: Adjust confidence thresholds based on environmental noise and accuracy requirements
3. **Buffer Management**: Optimize audio buffer sizes for real-time processing without excessive latency
4. **Safety Validation**: Implement multiple safety checks to prevent inappropriate command execution
5. **Fallback Mechanisms**: Provide graceful fallback for low-confidence recognitions

### 8.2. Integration Considerations
When integrating this pipeline with humanoid robots:

1. **Modular Design**: Keep components modular to allow for substitution and updates
2. **Real-time Performance**: Ensure the pipeline operates within required time constraints
3. **Error Handling**: Implement robust error handling for each pipeline stage
4. **Security**: Validate and sanitize voice commands to prevent malicious inputs
5. **Privacy**: Consider privacy implications when processing voice data

## Summary

The speech-to-text processing pipeline for humanoid robotics encompasses multiple stages from audio capture to action execution. The pipeline must be carefully designed and configured to handle the real-time requirements of humanoid robotics applications while ensuring safety and accuracy. The integration of Whisper with ROS 2 navigation systems provides a robust foundation for voice-controlled robot navigation and interaction.

## References

1. Radford, A., et al. (2022). "Robust Speech Recognition via Large-Scale Weak Supervision." arXiv preprint arXiv:2212.04356.

2. Macenski, S., et al. (2022). "Navigation2: A Navigation Framework for Ground Mobile Robots in ROS 2." Journal of Open Source Software.

3. Quigley, M., et al. (2009). "ROS: an open-source Robot Operating System." ICRA Workshop on Open Source Software.

4. Montemerlo, M., et al. (2003). "FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem." AAAI National Conference on Artificial Intelligence.

5. Fox, D., et al. (1997). "The Dynamic Window Approach to Collision Avoidance." IEEE Robotics & Automation Magazine.

6. Gerkey, B., et al. (2003). "The Player/Stage Project: Tools for Multi-Robot and Distributed Sensor Systems." International Conference on Advanced Robotics.

7. Siciliano, B., & Khatib, O. (2016). "Springer Handbook of Robotics." Springer.

8. Thrun, S., Burgard, W., & Fox, D. (2005). "Probabilistic Robotics." MIT Press.

9. NVIDIA. (2024). "Isaac Sim Documentation: Perception Simulation." NVIDIA Developer.

10. Open Source Robotics Foundation. (2023). "Gazebo Simulation Tutorial." Gazebo Documentation.

11. Vaswani, A., et al. (2017). "Attention Is All You Need." Advances in Neural Information Processing Systems.

12. Devlin, J., et al. (2019). "BERT: Pre-training of Deep Bidirectional Transformers for Language Understanding." North American Chapter of the Association for Computational Linguistics.