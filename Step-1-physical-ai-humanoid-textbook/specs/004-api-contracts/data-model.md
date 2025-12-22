# Data Model: API Contracts for Frontend Integration

## Overview
This document outlines the data models required for implementing API contracts that enable frontend integration with the RAG backend for Physical AI & Humanoid Robotics textbook.

## Core Entities

### QueryRequest
**Purpose**: Represents questions submitted by the frontend, containing question text and optional selected text context for additional context

**Fields**:
- `question` (string, required): The main question text from the user
- `selected_text` (string, optional): Additional context selected by the user
- `user_context` (object, optional): Additional contextual information from the user
- `metadata` (object, optional): Request metadata (timestamp, source, etc.)

**Validation Rules**:
- `question` must not be empty
- `question` length must be between 1 and 2000 characters
- `selected_text` length must be between 0 and 5000 characters if provided
- `user_context` and `metadata` must be valid JSON objects if provided

**Example**:
```json
{
  "question": "What are the key components of a humanoid robot?",
  "selected_text": "In chapter 3, the author discusses sensors and actuators...",
  "user_context": {
    "user_id": "user123",
    "session_id": "session456"
  },
  "metadata": {
    "timestamp": "2025-12-19T10:00:00Z",
    "source": "textbook_reader"
  }
}
```

### AgentResponse
**Purpose**: Contains the RAG agent's answer, source citations, confidence score, and token usage statistics

**Fields**:
- `answer` (string, required): The agent's response to the user's question
- `sources` (array of objects, required): List of sources referenced in the answer
  - `document_id` (string): Unique identifier for the source document
  - `page_number` (integer, optional): Page where the information appears
  - `section_title` (string, optional): Section where the information appears
  - `excerpt` (string, optional): Relevant excerpt from the source
- `confidence` (float, required): Confidence score for the response (0-1)
- `usage_stats` (object, optional): Token usage statistics
  - `prompt_tokens` (integer): Number of tokens in the prompt
  - `completion_tokens` (integer): Number of tokens in the completion
  - `total_tokens` (integer): Total number of tokens used

**Validation Rules**:
- `answer` must not be empty
- `sources` array must contain valid objects with required fields
- `confidence` must be between 0 and 1
- `usage_stats` values must be non-negative integers if provided

**Example**:
```json
{
  "answer": "Humanoid robots typically consist of a head, torso, arms, and legs with multiple joints and actuators to enable human-like movement...",
  "sources": [
    {
      "document_id": "doc001",
      "page_number": 45,
      "section_title": "Basic Components",
      "excerpt": "The fundamental components of humanoid robots include a head with sensors, a torso with actuators, and limbs with joints..."
    }
  ],
  "confidence": 0.92,
  "usage_stats": {
    "prompt_tokens": 120,
    "completion_tokens": 85,
    "total_tokens": 205
  }
}
```

### ErrorResponse
**Purpose**: Standardized error responses for frontend consumption with error codes and human-readable messages

**Fields**:
- `error` (string, required): Error code (e.g., VALIDATION_ERROR, INTERNAL_ERROR)
- `message` (string, required): Human-readable error message
- `details` (object, optional): Additional error details for debugging

**Validation Rules**:
- `error` must be a valid error code string
- `message` must not be empty
- `details` must be a valid JSON object if provided

**Example**:
```json
{
  "error": "VALIDATION_ERROR",
  "message": "Question field is required and cannot be empty",
  "details": {
    "field": "question",
    "value": "",
    "constraint": "min_length=1"
  }
}
```

### API Endpoint
**Purpose**: Documented RESTful endpoints that the frontend can call with predictable request/response schemas

**Fields**:
- `path` (string, required): The endpoint path (e.g., "/api/v1/rag/query")
- `method` (string, required): HTTP method (e.g., "POST")
- `request_model` (string, required): Pydantic model for request validation
- `response_model` (string, required): Pydantic model for response validation
- `description` (string, required): Brief description of the endpoint's purpose

**Validation Rules**:
- `path` must follow REST conventions
- `method` must be a valid HTTP method
- Request and response models must be defined
- Description must clearly explain the endpoint's function

## State Transitions

For the API integration layer, the state flow is primarily request-response based:
1. QueryRequest received → Validation → Processing
2. Processing → RAG System Query → Response Formatting
3. Response Formatting → AgentResponse returned

## Relationships

- A `QueryRequest` generates one `AgentResponse` or `ErrorResponse`
- An `AgentResponse` references multiple `sources` which link to the knowledge base documents
- `ErrorResponse` contains information about why a `QueryRequest` could not be processed
- `API Endpoint` processes `QueryRequest` and returns either `AgentResponse` or `ErrorResponse`