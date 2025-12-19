from fastapi import APIRouter, HTTPException, Request
import time
from typing import Dict, Any
from src.agents.rag_agent import RAGAgent
from src.api.models import QueryRequest, AgentResponse, ErrorResponse, BaseModel
from pydantic import Field
from src.config.settings import settings

router = APIRouter(prefix="/rag", tags=["RAG Agent"])


class QueryValidationResponse(BaseModel):
    """
    Response model for query validation endpoint
    """
    valid: bool = Field(..., description="Whether the query is valid")
    message: str = Field(..., description="Validation result message")


@router.post(
    "/query",
    response_model=AgentResponse,
    responses={
        200: {"description": "Successful response with answer from the RAG agent"},
        400: {"model": ErrorResponse, "description": "Bad request - invalid input parameters"},
        422: {"model": ErrorResponse, "description": "Unprocessable entity - validation error"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Submit a question to the RAG agent",
    description="Allows users to ask questions and get answers based strictly on the Physical AI & Humanoid Robotics textbook content. The agent will answer strictly from retrieved context, supporting both normal questions and selected-text questions."
)
async def query_rag_agent(request: QueryRequest):
    """
    Submit a question to the RAG agent and get an answer based strictly on the textbook content

    - **question**: The main question text from the user (required, 1-2000 characters)
    - **selected_text**: Additional context selected by the user (optional, up to 5000 characters)
    - **user_context**: Additional contextual information from the user (optional)
    - **metadata**: Request metadata (optional)

    The response will include:
    - **answer**: The agent's response to the user's question
    - **sources**: List of sources referenced in the answer
    - **confidence**: Agent's confidence level in the response (0-1)
    - **usage_stats**: Token usage statistics (optional)
    """
    try:
        # Initialize the RAG agent
        rag_agent = RAGAgent()

        # Process the query using the RAG agent
        result = await rag_agent.process_query(
            question=request.question,
            selected_text=request.selected_text,
            user_context=request.user_context
        )

        # Create and return the response
        response = AgentResponse(**result)

        # Validate the response
        if not rag_agent.response_formatter.validate_agent_response(result):
            raise HTTPException(
                status_code=500,
                detail="Error validating agent response format"
            )

        return response

    except HTTPException as http_ex:
        # Re-raise HTTP exceptions as-is, they already have appropriate status codes
        raise
    except Exception as e:
        # Log the error for debugging
        import logging
        logging.error(f"Error in RAG query endpoint: {str(e)}", exc_info=True)

        # Raise a 500 error for any unhandled exceptions
        raise HTTPException(
            status_code=500,
            detail="Internal server error occurred while processing the query"
        )


@router.post(
    "/query/validate",
    response_model=QueryValidationResponse,
    responses={
        200: {"description": "Validation result"},
        422: {"model": ErrorResponse, "description": "Unprocessable entity - validation error"},
        500: {"model": ErrorResponse, "description": "Internal server error"}
    },
    summary="Validate a query without processing it",
    description="Allows external systems to validate a query before fully processing it, useful for client-side validation."
)
async def validate_query(request: QueryRequest):
    """
    Validate a query without processing it, useful for external systems to check query format
    """
    try:
        # If we get here, the Pydantic model validation has already passed
        # We can perform additional custom validations if needed
        if not request.question or len(request.question.strip()) == 0:
            return QueryValidationResponse(
                valid=False,
                message="Question field is required and cannot be empty"
            )

        if len(request.question) > 2000:
            return QueryValidationResponse(
                valid=False,
                message="Question length exceeds maximum allowed characters (2000)"
            )

        if request.selected_text and len(request.selected_text) > 5000:
            return QueryValidationResponse(
                valid=False,
                message="Selected text length exceeds maximum allowed characters (5000)"
            )

        # If all validations pass
        return QueryValidationResponse(
            valid=True,
            message="Query is valid"
        )

    except Exception as e:
        import logging
        logging.error(f"Error in query validation endpoint: {str(e)}", exc_info=True)

        raise HTTPException(
            status_code=500,
            detail="Internal server error occurred while validating the query"
        )


@router.get(
    "/health",
    summary="Health check endpoint",
    description="Check if the RAG service is running and healthy"
)
async def health_check():
    """
    Health check endpoint to verify the service is running
    """
    return {"status": "healthy", "service": "RAG Agent API"}