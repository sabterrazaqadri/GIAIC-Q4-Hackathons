# Quickstart: Agent + FastAPI RAG Service

## Overview
This guide will help you quickly set up and run the RAG-enabled agent service that answers questions based on the Physical AI & Humanoid Robotics textbook content.

## Prerequisites
- Python 3.11 or higher
- pip package manager
- OpenAI API key
- Access to the Spec-2 retrieval system

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Navigate to the backend directory:
```bash
cd Backend
```

3. Create and activate a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

4. Install dependencies:
```bash
pip install -r requirements.txt
```

5. Set up environment variables:
```bash
cp .env.example .env
# Edit .env and add your OpenAI API key
```

## Running the Service

1. Start the FastAPI server:
```bash
uvicorn src.api.main:app --reload
```

2. The API will be available at `http://localhost:8000`

3. The API documentation will be available at `http://localhost:8000/docs`

## Testing the API

1. Example cURL request:
```bash
curl -X POST "http://localhost:8000/api/rag/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the key components of a humanoid robot?",
    "selected_text": "In chapter 3, the author discusses sensors and actuators..."
  }'
```

2. You can also use the API documentation UI at `http://localhost:8000/docs` to test the endpoints interactively.

## Configuration

The service can be configured through environment variables in the `.env` file:
- `OPENAI_API_KEY`: Your OpenAI API key
- `RETRIEVAL_API_URL`: URL of the Spec-2 retrieval system
- `LOG_LEVEL`: Logging level (default: INFO)

## Development

1. The project follows this structure:
```
Backend/
├── src/
│   ├── agents/          # RAG agent implementation
│   ├── api/             # FastAPI endpoints
│   └── services/        # Supporting services
├── tests/               # Unit and integration tests
└── requirements.txt     # Python dependencies
```

2. To run tests:
```bash
pytest
```

3. To run only unit tests:
```bash
pytest tests/unit/
```

4. To run only integration tests:
```bash
pytest tests/integration/
```

## Troubleshooting

1. If you get OpenAI API errors, verify your API key is correct in the `.env` file.

2. If the retrieval service is not working, check that the `RETRIEVAL_API_URL` is correctly set in the `.env` file.

3. For any dependency issues, try:
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```