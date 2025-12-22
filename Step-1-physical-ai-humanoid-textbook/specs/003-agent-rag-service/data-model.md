# Data Model: Agent + FastAPI RAG Service

## Overview
This document outlines the data models required for implementing the RAG-enabled agent using OpenAI Agents SDK with integrated retrieval functionality exposed via FastAPI API.

## Core Entities

### Query Request
**Purpose**: Represents questions submitted to the RAG agent, containing the question text and optional selected text context

**Fields**:
- `question` (string, required): The main question text from the user
- `selected_text` (string, optional): Additional context selected by the user
- `user_context` (object, optional): Additional contextual information from the user
- `metadata` (object, optional): Request metadata (timestamp, source, etc.)

**Validation Rules**:
- `question` must not be empty
- `question` length must be between 1 and 2000 characters
- `selected_text` length must be between 0 and 5000 characters if provided

### Retrieved Context
**Purpose**: Contains documents and information retrieved from the knowledge base that are relevant to the user's query

**Fields**:
- `documents` (array of strings, required): The retrieved document snippets
- `relevance_scores` (array of floats, required): Relevance scores for each document
- `sources` (array of objects, required): Source information for each document
  - `document_id` (string): Unique identifier for the source document
  - `page_number` (integer, optional): Page where the content appears
  - `section_title` (string, optional): Section where the content appears

**Validation Rules**:
- `documents` array must not be empty
- `relevance_scores` array length must match `documents` array length
- All relevance scores must be between 0 and 1

### Agent Response
**Purpose**: The final output from the agent, containing the answer and references to the sources used

**Fields**:
- `answer` (string, required): The agent's response to the user's question
- `sources` (array of objects, required): List of sources referenced in the answer
  - `document_id` (string): Unique identifier for the source document
  - `page_number` (integer, optional): Page where the information appears
  - `section_title` (string, optional): Section where the information appears
  - `excerpt` (string, optional): Relevant excerpt from the source
- `confidence` (float, required): Agent's confidence level in the response (0-1)
- `usage_stats` (object, optional): Token usage statistics
  - `prompt_tokens` (integer): Number of tokens in the prompt
  - `completion_tokens` (integer): Number of tokens in the completion
  - `total_tokens` (integer): Total number of tokens used

**Validation Rules**:
- `answer` must not be empty
- `confidence` must be between 0 and 1
- Sources array may be empty but must be present

### API Endpoint
**Purpose**: Represents the FastAPI endpoints that accept requests and return responses from the agent

**Fields**:
- `path` (string, required): The endpoint path (e.g., "/api/rag/query")
- `method` (string, required): HTTP method (e.g., "POST")
- `request_model` (string, required): Pydantic model for request validation
- `response_model` (string, required): Pydantic model for response validation
- `description` (string, required): Brief description of the endpoint's purpose

**Validation Rules**:
- `path` must follow REST conventions
- `method` must be a valid HTTP method
- Request and response models must be defined

## State Transitions (if applicable)

For the RAG service, we have the following state flow:
1. Query Request received → Validation → Processing
2. Processing → Context Retrieval → Agent Response Generation
3. Agent Response Generation → Response Formatting → Response Returned

## Relationships

- A `Query Request` generates one `Agent Response`
- An `Agent Response` references multiple `Retrieved Context` items
- `Retrieved Context` items link to source documents in the knowledge base
- `API Endpoint` processes `Query Request` and returns `Agent Response`