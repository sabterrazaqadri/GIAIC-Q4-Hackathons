# Physical-AI & Humanoid-Robotics — AI-Native Textbook + Embedded RAG Chatbot Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-19

## Active Technologies

- Python 3.11
- OpenAI Agents SDK
- FastAPI
- uvicorn
- spec-2 retrieval module
- pytest

## Project Structure

```text
Backend/
├── src/
│   ├── agents/
│   │   ├── rag_agent.py          # Main RAG agent implementation
│   │   ├── agent_tools.py        # Tools for the agent (including retrieval)
│   │   └── __init__.py
│   ├── api/
│   │   ├── main.py              # FastAPI application
│   │   ├── routes/
│   │   │   └── rag.py           # RAG endpoints
│   │   └── __init__.py
│   ├── services/
│   │   ├── retrieval_service.py # Integration with spec-2 retrieval
│   │   ├── response_formatter.py # Format agent responses
│   │   └── __init__.py
│   └── __init__.py
├── tests/
│   ├── unit/
│   │   ├── test_agents/
│   │   ├── test_api/
│   │   └── test_services/
│   ├── integration/
│   │   └── test_end_to_end.py
│   └── __init__.py
├── requirements.txt
└── config/
    └── settings.py
```

## Commands

- `uvicorn Backend.src.api.main:app --reload` - Start the development server
- `pytest` - Run all tests
- `pytest tests/unit/` - Run unit tests only
- `pytest tests/integration/` - Run integration tests only

## Code Style

Python code should follow PEP 8 standards:
- Use 4 spaces for indentation
- Maximum line length of 88 characters
- Use descriptive variable names
- Include docstrings for functions and classes
- Use type hints for function parameters and return values

## Recent Changes

- 003-agent-rag-service: Implementation of RAG-enabled agent using OpenAI Agents SDK, with retrieval as a tool and FastAPI API exposure
- 002-ros2-fundamentals: Content about ROS2 fundamentals
- 001-physical-ai-modules: Initial physical AI modules

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->