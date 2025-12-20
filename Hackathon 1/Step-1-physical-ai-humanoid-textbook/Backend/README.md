# RAG Agent Backend for Physical-AI & Humanoid Robotics Textbook

This project implements a Retrieval-Augmented Generation (RAG) agent backend for the Physical-AI & Humanoid Robotics Textbook. It provides a conversational interface that allows users to ask questions about the textbook content and receive answers grounded in the source material.

The system features:

- **RAG Agent**: An OpenAI-powered agent that answers questions based on textbook content
- **Context Handling**: Support for questions that reference selected text
- **Source Attribution**: Responses include citations to source documents
- **API Integration**: RESTful API for external system integration
- **Performance Monitoring**: Built-in metrics and response time tracking
- **Security**: Multiple layers of security controls and validation

## Setup

1. Install dependencies: `pip install -r requirements.txt`
2. Copy `.env.example` to `.env` and fill in your credentials
3. Run the application: `uvicorn src.api.main:app --reload`

## API Endpoints

### RAG Agent Endpoints
- `POST /api/v1/rag/query` - Submit a question to the RAG agent
- `POST /api/v1/rag/query/validate` - Validate a query without processing it

### System Endpoints
- `GET /` - Health check
- `GET /health` - Health check
- `GET /metrics` - Performance metrics
- `GET /docs` - API Documentation (Swagger UI)
- `GET /redoc` - API Documentation (ReDoc)

## Environment Variables

- `OPENAI_API_KEY` - Your OpenAI API key for the RAG agent
- `COHERE_API_KEY` - Your Cohere API key for embedding generation
- `QDRANT_URL` - URL for your Qdrant instance
- `QDRANT_API_KEY` - API key for your Qdrant instance (if required)
- `QDRANT_COLLECTION_NAME` - Name of the collection to store vectors (default: textbook_chunks)
- `CHUNK_SIZE` - Size of content chunks in tokens (default: 512)
- `CHUNK_OVERLAP` - Number of overlapping tokens between chunks (default: 50)
- `CRAWL_DELAY` - Delay between requests in seconds (default: 1.0)
- `MAX_RETRIES` - Maximum number of retry attempts for failed requests (default: 3)
- `TIMEOUT` - Request timeout in seconds (default: 30)
- `HOST` - Host to run the application on (default: 0.0.0.0)
- `PORT` - Port to run the application on (default: 8000)
- `RELOAD` - Enable auto-reload on code changes (default: True)

## Request Format

Send requests to `/api/v1/rag/query` with the following format:

```json
{
  "question": "Your question about the textbook content",
  "selected_text": "Optional selected text to provide additional context",
  "user_context": {
    "additional": "contextual information"
  },
  "metadata": {
    "request_id": "optional request identifier"
  }
}
```

## Response Format

The API returns responses in the following format:

```json
{
  "answer": "The agent's response to your question",
  "sources": [
    {
      "document_id": "unique identifier for the source document",
      "page_number": 45,
      "section_title": "Section where the information appears",
      "excerpt": "Relevant excerpt from the source"
    }
  ],
  "confidence": 0.95,
  "usage_stats": {
    "prompt_tokens": 120,
    "completion_tokens": 85,
    "total_tokens": 205
  }
}
```

## Architecture

The system follows a modular service architecture:

- **RAGAgent**: Main agent that processes user queries using OpenAI
- **AgentToolsService**: Provides tools for the agent, including retrieval
- **RetrievalService**: Retrieves relevant context from the knowledge base
- **ResponseFormatterService**: Formats agent responses according to specifications
- **FastAPI**: Web framework providing the API layer and documentation

## Security

This implementation includes multiple layers of security:

- **CORS**: Configured with specific allowed origins (not wildcard in production)
- **Security Headers**: X-Content-Type-Options, X-Frame-Options, XSS protection
- **Input Validation**: Comprehensive validation of all request parameters
- **Error Handling**: Structured error responses without sensitive information leak

## Performance Monitoring

- **Response Time Headers**: X-Response-Time and X-Server-Process-Time headers
- **Slow Request Logging**: Requests taking longer than 2 seconds are logged as warnings
- **Metrics Endpoint**: `/metrics` endpoint for monitoring systems

## Deployment

### Using Docker

1. Build the Docker image:
   ```
   docker build -t rag-agent-backend .
   ```

2. Run the container:
   ```
   docker run -d -p 8000:8000 --env-file .env rag-agent-backend
   ```

### Direct Deployment

1. Install Python dependencies:
   ```
   pip install -r requirements.txt
   ```

2. Set up environment variables in a `.env` file based on `.env.example`

3. Run the application:
   ```
   uvicorn src.api.main:app --host 0.0.0.0 --port 8000
   ```

## Testing

Run the tests using pytest:
```
pytest tests/
```

For specific test modules:
```
pytest tests/test_crawler.py
pytest tests/test_chunking.py
pytest tests/test_embedding.py
pytest tests/test_storage.py
pytest tests/test_integration.py
pytest tests/integration/test_end_to_end.py
```

## Development

The project is organized as follows:

```
Backend/
├── src/
│   ├── agents/          # RAG agent implementation
│   ├── api/             # FastAPI endpoints
│   │   ├── middleware/  # Security and other middleware
│   │   └── routes/      # API route definitions
│   ├── config/          # Configuration settings
│   ├── models/          # Data models
│   ├── services/        # Business logic services
│   └── utils/           # Utility functions
├── tests/               # Unit and integration tests
│   ├── integration/     # Integration tests
│   ├── performance/     # Performance tests
│   └── unit/            # Unit tests
├── docs/                # Documentation
└── requirements.txt     # Python dependencies
```

## Additional Documentation

For more detailed documentation, see:
- [API Documentation](./docs/api.md) - Complete API reference
- [Performance Tests](./tests/performance/test_performance.py) - Performance test suite
- [Integration Tests](./tests/integration/test_end_to_end.py) - End-to-end test scenarios