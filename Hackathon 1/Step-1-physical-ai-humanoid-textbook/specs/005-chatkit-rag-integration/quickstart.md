# Quickstart Guide: ChatKit UI Integration for RAG System

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Docker (optional, for containerized deployment)
- OpenAI API key
- Access to Qdrant vector database
- Access to Postgres database

## Setup Instructions

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Set Up Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

If no requirements.txt exists yet, install the core dependencies:

```bash
pip install fastapi uvicorn openai python-multipart sqlalchemy async-sqlalchemy pydantic qdrant-client
```

### 4. Configure Environment Variables

Create a `.env` file in the project root with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
DATABASE_URL=postgresql+asyncpg://username:password@localhost/dbname
```

### 5. Run Database Migrations (if applicable)

```bash
alembic upgrade head
```

### 6. Start the Development Server

```bash
uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`.

## API Usage Example

### Send a Chat Request

```bash
curl -X POST http://localhost:8000/chat/completions \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer your-token-if-required" \
  -d '{
    "messages": [
      {
        "role": "user",
        "content": "Explain the concept of inverse kinematics in humanoid robotics"
      }
    ],
    "selected_text": "Inverse kinematics is the mathematical process of calculating the joint angles...",
    "temperature": 0.7
  }'
```

## Running Tests

```bash
# Run all tests
pytest

# Run tests with coverage
pytest --cov=backend

# Run specific test file
pytest tests/unit/test_chat.py
```

## Docker Deployment

### Build the Docker Image

```bash
docker build -t chatkit-rag-integration .
```

### Run the Container

```bash
docker run -d -p 8000:8000 \
  -e OPENAI_API_KEY=your_openai_api_key_here \
  -e QDRANT_URL=your_qdrant_url_here \
  -e DATABASE_URL=postgresql+asyncpg://username:password@host:port/dbname \
  chatkit-rag-integration
```

## Key Endpoints

- `POST /chat/completions` - Main endpoint for chat interactions with RAG
- `POST /chat/validate` - Validate queries against available content
- `GET /docs` - Interactive API documentation (Swagger UI)
- `GET /redoc` - Alternative API documentation (ReDoc)

## Development Guidelines

1. All new features should include unit and integration tests
2. Follow the existing code structure and naming conventions
3. Document all API endpoints with proper OpenAPI specifications
4. Use Pydantic models for request/response validation
5. Implement proper error handling and logging
6. Ensure all responses are grounded in textbook content without hallucinations