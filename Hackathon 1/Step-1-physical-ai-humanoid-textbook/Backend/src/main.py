from fastapi import FastAPI, HTTPException
from src.api.v1.endpoints import router as api_router
import src.config.settings as settings
from src.utils.logging import setup_logging

# Initialize the FastAPI app
app = FastAPI(
    title="RAG Backend for Physical-AI & Humanoid Robotics Textbook",
    description="Backend service for crawling, chunking, embedding, and storing textbook content",
    version="1.0.0"
)

# Include the API router
app.include_router(api_router, prefix="/api/v1")

@app.get("/")
def read_root():
    return {"message": "RAG Backend for Physical-AI & Humanoid Robotics Textbook"}

@app.on_event("startup")
async def startup_event():
    # Initialize logging
    logger = setup_logging()

    # Validate configuration
    try:
        # Check that required settings are present
        if not settings.cohere_api_key:
            raise ValueError("COHERE_API_KEY is required")

        if not settings.qdrant_url:
            raise ValueError("QDRANT_URL is required")

        logger.info("Configuration validated successfully")

        # Additional startup tasks can go here
        logger.info("Application starting up...")
    except Exception as e:
        logger.error(f"Startup failed: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Startup failed: {str(e)}")

@app.on_event("shutdown")
async def shutdown_event():
    # Cleanup resources
    print("Application shutting down...")