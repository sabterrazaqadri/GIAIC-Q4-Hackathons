# Research: API Contracts for Frontend Integration

## Overview
This document summarizes the research and decisions made for implementing API contracts for frontend integration in the Physical AI & Humanoid Robotics textbook RAG system.

## Decision: FastAPI Framework for Backend API
**Rationale**: The feature specification requires a backend that allows frontend communication without requiring configuration changes. FastAPI provides excellent support for creating clean, documented APIs with automatic OpenAPI schema generation, which aligns with the requirement for "clean JSON contracts documented".

**Alternatives considered**:
- Flask: Simpler but less automatic documentation
- Django REST Framework: More complex, overkill for this use case
- Express.js: Would require switching to Node.js ecosystem

**Decision**: Use FastAPI as it provides automatic API documentation, Pydantic-based request/response validation, and async support which is important for handling AI API calls efficiently.

## Decision: RESTful API Design Pattern
**Rationale**: The feature requires stable, well-documented endpoints that frontend developers can integrate with easily. RESTful patterns provide predictability and broad tooling support.

**Alternatives considered**:
- GraphQL: More flexible queries but more complex for this use case
- gRPC: Higher performance but more complex for frontend integration
- Custom protocol: Would require more frontend development work

**Decision**: Use RESTful API design with a single primary endpoint that handles both general queries and queries with selected text via optional parameters.

## Decision: Single Endpoint for Both Query Types
**Rationale**: The spec mentions handling both general queries and queries with selected text context. A single endpoint with optional selected text parameter provides cleaner integration compared to separate endpoints.

**Alternatives considered**:
- Separate endpoints (/api/query-general, /api/query-selected-text): Would require frontend to know which endpoint to call
- Different HTTP methods: GET vs POST would not be semantically correct
- Different request bodies with same endpoint: Could work, but optional parameter is cleaner

**Decision**: Use a single POST endpoint with an optional selected_text field in the request body.

## Decision: JSON Schema for Request/Response
**Rationale**: The spec requires "clean JSON contracts" and "structured responses with answers, source citations, and confidence indicators". Well-defined JSON schemas ensure frontend-backend compatibility.

**Alternatives considered**:
- No schemas: Would lead to integration issues
- Custom format: Would be harder to document and maintain
- XML or other formats: JSON is standard for web APIs

**Decision**: Define clear JSON schemas for requests and responses using Pydantic models for automatic validation and documentation.

## Decision: CORS Configuration for Localhost Development
**Rationale**: The feature requires "frontend communication without requiring configuration changes" and "localhost integration". Proper CORS configuration is essential.

**Alternatives considered**:
- No CORS configuration: Would block frontend requests
- Wildcard CORS: Security risk in production
- Strict origin matching: Would require configuration changes

**Decision**: Implement configurable CORS that allows localhost origins in development but can be restricted in production.

## Decision: Error Handling Strategy
**Rationale**: The spec requires the system to "handle communication timeouts gracefully" and "return appropriate error responses".

**Alternatives considered**:
- Generic error responses: Would not provide helpful debug information
- Detailed stack traces: Would be a security risk
- HTTP status codes only: Would not provide enough context

**Decision**: Use structured error responses with appropriate HTTP status codes and meaningful error messages that help frontend developers handle errors appropriately.

## Decision: Request Validation Approach
**Rationale**: The system must "validate user inputs" and "validate query format before processing".

**Alternatives considered**:
- Client-side validation only: Insecure and unreliable
- Minimal validation: Would lead to processing errors
- Complex validation rules: Might slow down requests

**Decision**: Implement comprehensive server-side validation using Pydantic models with specific constraints for question length, content, and selected text limitations.

## Technology Stack Summary
- **Backend**: Python 3.11 with FastAPI
- **API Documentation**: Automatic OpenAPI/Swagger via FastAPI
- **Request/Response Validation**: Pydantic models
- **Async handling**: FastAPI's built-in async support for handling AI API calls
- **CORS**: FastAPI middleware for cross-origin requests
- **Error handling**: Custom exception handlers with structured responses