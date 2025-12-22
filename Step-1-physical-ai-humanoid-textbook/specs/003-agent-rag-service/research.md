# Research: Agent + FastAPI RAG Service

## Overview
This document summarizes the research and decisions made for implementing the RAG-enabled agent using OpenAI Agents SDK integrated with retrieval functionality and exposed via FastAPI API.

## Decision: OpenAI Agents SDK Implementation Approach
**Rationale**: The feature specification requires building a RAG-enabled agent using OpenAI Agents SDK. Research shows that the OpenAI Assistant API is the recommended way to create agents that can use tools and respond to user queries based on retrieved context.

**Alternatives considered**:
- Using OpenAI's Completions API directly with manual context injection
- Using LangChain agents with custom tools
- Using Azure OpenAI Service instead of OpenAI API

**Decision**: Use OpenAI Assistant API through the OpenAI Python SDK as it provides native support for tools, maintains conversation state, and enables the RAG functionality required.

## Decision: Retrieval Integration as Tool
**Rationale**: The specification requires integrating retrieval functionality as a tool within the agent. The OpenAI Assistant API supports custom tools that can be called during agent execution.

**Alternatives considered**:
- Pre-retrieving context and injecting it into the prompt
- Using OpenAI's built-in file search functionality
- Creating a custom retrieval function that the agent can call

**Decision**: Implement retrieval as a custom tool that the agent can call when it needs additional context. This provides the flexibility required and maintains clear separation between the agent and retrieval components.

## Decision: FastAPI for API Exposure
**Rationale**: The specification requires exposing agent functionality via FastAPI. FastAPI provides excellent support for async operations, automatic API documentation, and integration with OpenAI SDK.

**Alternatives considered**:
- Flask for simpler synchronous operations
- Django REST Framework for more complex applications
- Express.js for Node.js implementation

**Decision**: Use FastAPI with async endpoints to handle agent requests efficiently and provide automatic OpenAPI documentation.

## Decision: Handling Normal Questions vs Selected-Text Questions
**Rationale**: The system must support both normal questions and questions with selected text context. The approach needs to conditionally include the selected text in the agent's context.

**Alternatives considered**:
- Using separate endpoints for different question types
- Using a single endpoint with conditional logic
- Preprocessing all questions to include context uniformly

**Decision**: Use a single endpoint with a `selected_text` optional parameter. When present, the agent will incorporate this text with the retrieved knowledge base information.

## Decision: State Management and Context Limiting
**Rationale**: The specification states "No memory beyond request scope." Each API call needs to be self-contained without relying on previous interactions.

**Alternatives considered**:
- Using the Assistant API's built-in thread management
- Implementing server-side session storage
- Passing full context in each request

**Decision**: Create a new thread for each request to ensure no memory persistence beyond the request scope. This aligns with the constraint in the specification.

## Decision: Response Grounding Verification
**Rationale**: The agent must answer strictly from retrieved context. We need to ensure the agent doesn't hallucinate information.

**Alternatives considered**:
- Post-processing responses to remove hallucinated content
- Using specific prompting techniques to limit hallucinations
- Implementing a verification step that checks responses against retrieved context

**Decision**: Combine proper prompting techniques with verification by ensuring the agent only uses information from the retrieval tool, which is populated with verified textbook content.

## Technology Stack Summary
- **Backend**: Python 3.11 with FastAPI
- **AI Agent**: OpenAI Assistant API (part of OpenAI SDK)
- **Retrieval System**: Integration with existing Spec-2 retrieval as a tool
- **Async Handling**: FastAPI's built-in async support
- **API Documentation**: Automatic OpenAPI documentation via FastAPI