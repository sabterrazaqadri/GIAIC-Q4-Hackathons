# Quickstart: API Contracts for Frontend Integration

## Overview
This guide will help you quickly understand and integrate with the RAG backend API for the Physical AI & Humanoid Robotics textbook.

## API Base URL
The API is available at:
- Development: `http://localhost:8000`
- Production: `https://api.example.com` (to be determined)

## Authentication
The API does not require authentication for this initial implementation.

## Making Your First Request

### Endpoint: Query the Textbook
Use the `/api/v1/rag/query` endpoint to ask questions about the textbook content:

```bash
curl -X POST "http://localhost:8000/api/v1/rag/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key components of a humanoid robot?",
    "selected_text": "In chapter 3, the author discusses sensors and actuators..."
  }'
```

### Request Format
The request body should contain:

- `question` (required, string): Your question about the textbook
- `selected_text` (optional, string): Additional context from the textbook
- `user_context` (optional, object): Additional contextual information
- `metadata` (optional, object): Request metadata

### Response Format
The API will return a response with:

- `answer` (string): The AI-generated answer to your question
- `sources` (array): List of sources used to generate the answer
- `confidence` (number): Confidence score between 0 and 1
- `usage_stats` (object): Token usage statistics (optional)

## Validating Requests
Before making a full query, you can validate your request format using:

```bash
curl -X POST "http://localhost:8000/api/v1/rag/query/validate" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key components of a humanoid robot?"
  }'
```

## Error Handling
The API returns structured error responses with:

- `error`: A code identifying the error type
- `message`: A human-readable error message
- `details`: Additional information for debugging (optional)

Common error codes:
- `VALIDATION_ERROR`: Request parameters failed validation
- `INTERNAL_ERROR`: An unexpected error occurred on the server
- `TIMEOUT_ERROR`: Request took too long to process

## Frontend Integration Example

Here's a simple JavaScript example for frontend integration:

```javascript
async function queryTextbook(question, selectedText = null) {
  const requestBody = {
    question: question
  };
  
  if (selectedText) {
    requestBody.selected_text = selectedText;
  }
  
  try {
    const response = await fetch('http://localhost:8000/api/v1/rag/query', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(requestBody)
    });
    
    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.message);
    }
    
    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error querying textbook:', error);
    throw error;
  }
}

// Usage
queryTextbook("What are the main components of a humanoid robot?")
  .then(result => console.log(result.answer))
  .catch(error => console.error(error));
```

## CORS Configuration
The API is configured to allow requests from localhost origins, making it easy to integrate with frontend applications during development.

## Performance Expectations
- 95% of requests should return within 500ms
- Responses are optimized for frontend consumption with clean JSON structure
- Proper error handling ensures the frontend can gracefully handle issues

## Troubleshooting
- If you receive a 400 error, check that your question is not empty and meets length requirements
- If you receive a 500 error, the backend may be temporarily unavailable; try again later
- Ensure your selected_text, if provided, is not excessively long (max 5000 characters)