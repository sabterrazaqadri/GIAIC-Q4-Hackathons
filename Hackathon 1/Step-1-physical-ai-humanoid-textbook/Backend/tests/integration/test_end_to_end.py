import pytest
import asyncio
from fastapi.testclient import TestClient
from src.api.main import app
from src.api.models import QueryRequest, AgentResponse

# Create test client
client = TestClient(app)


def test_user_story_1_basic_question():
    """
    Test User Story 1: RAG Question Answering
    End users submit questions to the RAG agent via the API, 
    and receive contextual answers based strictly on the Physical AI & Humanoid Robotics textbook content.
    """
    # Sample request for US1
    request_data = {
        "question": "What are the key components of a humanoid robot?",
        "selected_text": None
    }
    
    response = client.post("/api/v1/rag/query", json=request_data)
    
    # Assertions for US1
    assert response.status_code == 200
    
    # Parse response
    data = response.json()
    agent_response = AgentResponse(**data)
    
    # Validate response structure
    assert agent_response.answer is not None
    assert agent_response.confidence >= 0.0 and agent_response.confidence <= 1.0
    assert isinstance(agent_response.sources, list)
    assert hasattr(agent_response, 'usage_stats')  # Optional field


def test_user_story_1_with_selected_text():
    """
    Test User Story 1 with selected text (enhanced scenario)
    """
    # Sample request with selected text
    request_data = {
        "question": "What sensors are used for balance control?",
        "selected_text": "In chapter 3, the author discusses how humanoid robots use gyroscopes and accelerometers for balance control."
    }
    
    response = client.post("/api/v1/rag/query", json=request_data)
    
    # Assertions
    assert response.status_code == 200
    
    # Parse response
    data = response.json()
    agent_response = AgentResponse(**data)
    
    # Validate response structure
    assert agent_response.answer is not None
    assert agent_response.confidence >= 0.0 and agent_response.confidence <= 1.0
    assert isinstance(agent_response.sources, list)


def test_user_story_2_api_integration():
    """
    Test User Story 2: API Integration for External Systems
    External applications connect to the RAG service to enable their users 
    to ask questions about Physical AI & Humanoid Robotics topics.
    """
    # Test that the API provides consistent response format for external systems
    request_data = {
        "question": "Explain the ROS 2 communication model for humanoid robots.",
        "selected_text": "The ROS 2 framework provides real-time communication capabilities."
    }
    
    response = client.post("/api/v1/rag/query", json=request_data)
    
    # Assertions for US2
    assert response.status_code == 200
    
    # Check that the response follows the expected format
    data = response.json()
    agent_response = AgentResponse(**data)
    
    # Verify all required fields are present for external systems
    assert hasattr(agent_response, 'answer')
    assert hasattr(agent_response, 'sources')
    assert hasattr(agent_response, 'confidence')
    assert 0.0 <= agent_response.confidence <= 1.0
    
    # Test validation endpoint for external systems
    validation_response = client.post("/api/v1/rag/query/validate", json=request_data)
    assert validation_response.status_code == 200
    
    validation_data = validation_response.json()
    assert "valid" in validation_data
    assert "message" in validation_data


def test_user_story_3_selected_text_integration():
    """
    Test User Story 3: Specialized Content Retrieval
    Users ask questions that specifically reference selected text, 
    expecting answers that incorporate both the selected text and textbook knowledge.
    """
    # Sample request with selected text for US3
    request_data = {
        "question": "How does this relate to the control systems mentioned?",
        "selected_text": "The previous section mentioned using PID controllers for motor control in humanoid robots."
    }
    
    response = client.post("/api/v1/rag/query", json=request_data)
    
    # Assertions for US3
    assert response.status_code == 200
    
    # The response should incorporate both the selected text and textbook knowledge
    data = response.json()
    agent_response = AgentResponse(**data)
    
    assert agent_response.answer is not None
    # The response should logically connect to the selected text context
    assert isinstance(agent_response.sources, list)


def test_error_handling_scenarios():
    """
    Test error handling for robust API operation
    """
    # Test with empty question
    response = client.post("/api/v1/rag/query", json={
        "question": "",
        "selected_text": "Some context"
    })
    assert response.status_code == 422  # Validation error
    
    # Test with very long question
    long_question = "test " * 1000
    response = client.post("/api/v1/rag/query", json={
        "question": long_question,
        "selected_text": "Some context"
    })
    assert response.status_code == 422  # Validation error
    
    # Test validation endpoint with invalid data
    response = client.post("/api/v1/rag/query/validate", json={
        "question": ""
    })
    assert response.status_code == 200  # Should return validation result, not error
    validation_result = response.json()
    assert validation_result["valid"] is False


def test_health_endpoint():
    """
    Test health check endpoint
    """
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()
    
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"


if __name__ == "__main__":
    # Run the tests
    test_user_story_1_basic_question()
    test_user_story_1_with_selected_text()
    test_user_story_2_api_integration()
    test_user_story_3_selected_text_integration()
    test_error_handling_scenarios()
    test_health_endpoint()
    print("All end-to-end tests passed!")