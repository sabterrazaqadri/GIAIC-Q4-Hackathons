import requests
import xml.etree.ElementTree as ET
import trafilatura
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance, PointStruct
import cohere
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

# -------------------------------------
# CONFIG
# -------------------------------------
# Your Deployment Link:
SITEMAP_URL = "https://physical-ai-humanoid-textbook-mu.vercel.app/sitemap.xml"
COLLECTION_NAME = "physical_ai_humanoid_textbook"

cohere_client = cohere.Client("0DRQcyTI98p3HRpjuQv8tvg4IcQtRVBeublwiHoe")
EMBED_MODEL = "embed-english-v3.0"

# Connect to Qdrant Cloud
qdrant = QdrantClient(
    url="https://6eb3cc7d-3f4e-46a5-ae7c-20d8d583c238.europe-west3-0.gcp.cloud.qdrant.io:6333",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.4m9T9I-NlGbJF6KZG0edJ4FS2xfOoYMCSlGYVbv-Mss",
)

# Set up FastAPI app
app = FastAPI(title="Physical AI & Humanoid Robotics Chatbot API")

# Add CORS middleware to allow requests from the frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with your frontend URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------------------
# Helper functions for embeddings and retrieval
# -------------------------------------

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding


def retrieve(query: str) -> list[str]:
    embedding = get_embedding(query)
    result = qdrant.query_points(
        collection_name="textbook_content",  # Use the correct collection name
        query=embedding,
        limit=5
    )
    return [point.payload["content"] for point in result.points]


# Define request/response models
class ChatRequest(BaseModel):
    message: str


class ChatResponse(BaseModel):
    response: str

# -------------------------------------
# Step 1 — Extract URLs from sitemap
# -------------------------------------
def get_all_urls(sitemap_url):
    xml = requests.get(sitemap_url).text
    root = ET.fromstring(xml)

    urls = []
    for child in root:
        loc_tag = child.find("{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
        if loc_tag is not None:
            urls.append(loc_tag.text)

    print("\nFOUND URLS:")
    for u in urls:
        print(" -", u)

    return urls


# -------------------------------------
# Step 2 — Download page + extract text
# -------------------------------------
def extract_text_from_url(url):
    html = requests.get(url).text
    text = trafilatura.extract(html)

    if not text:
        print("[WARNING] No text extracted from:", url)

    return text


# -------------------------------------
# Step 3 — Chunk the text
# -------------------------------------
def chunk_text(text, max_chars=1200):
    chunks = []
    while len(text) > max_chars:
        split_pos = text[:max_chars].rfind(". ")
        if split_pos == -1:
            split_pos = max_chars
        chunks.append(text[:split_pos])
        text = text[split_pos:]
    chunks.append(text)
    return chunks


# -------------------------------------
# Step 4 — Create embedding
# -------------------------------------
def embed(text):
    response = cohere_client.embed(
        model=EMBED_MODEL,
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding


# -------------------------------------
# Step 5 — Store in Qdrant
# -------------------------------------
def create_collection():
    print("\nCreating Qdrant collection...")
    qdrant.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(
        size=1024,        # Cohere embed-english-v3.0 dimension
        distance=Distance.COSINE
        )
    )

def save_chunk_to_qdrant(chunk, chunk_id, url):
    vector = embed(chunk)

    qdrant.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            PointStruct(
                id=chunk_id,
                vector=vector,
                payload={
                    "url": url,
                    "text": chunk,
                    "chunk_id": chunk_id
                }
            )
        ]
    )


# -------------------------------------
# MAIN INGESTION PIPELINE
# -------------------------------------
def ingest_book():
    urls = get_all_urls(SITEMAP_URL)

    create_collection()

    global_id = 1

    for url in urls:
        print("\nProcessing:", url)
        text = extract_text_from_url(url)

        if not text:
            continue

        chunks = chunk_text(text)

        for ch in chunks:
            save_chunk_to_qdrant(ch, global_id, url)
            print(f"Saved chunk {global_id}")
            global_id += 1

    print("\n✔️ Ingestion completed!")
    print("Total chunks stored:", global_id - 1)


# -------------------------------------
# API Endpoints
# -------------------------------------

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        # Retrieve relevant documents based on the user's query
        retrieved_docs = retrieve(request.message)

        # Create a context from the retrieved documents
        context = "\n\n".join([f"Document {i+1}: {doc}" for i, doc in enumerate(retrieved_docs)])

        # Prepare the prompt for the LLM
        prompt = f"""
        You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
        Answer the user's question based only on the provided context.
        If the answer is not in the context, respond with "I don't know".

        Question: {request.message}

        Context:
        {context}
        """

        # Generate response using Cohere Chat API
        response = cohere_client.chat(
            model='command-r-plus-08-2024',  # Using a generative model
            message=request.message,
            documents=[{"title": f"Document {i}", "snippet": doc} for i, doc in enumerate(retrieved_docs, 1)],
            temperature=0.3,
        )

        return ChatResponse(response=response.text.strip())
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


# Import the new modular structure
from src.main import app as modular_app

# Define request/response models for backward compatibility
class ChatRequest(BaseModel):
    message: str


class ChatResponse(BaseModel):
    response: str


# Add the legacy /chat endpoint for backward compatibility
@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint_legacy(request: ChatRequest):
    """
    Legacy endpoint for backward compatibility.
    This endpoint will be phased out in favor of the new /chat/completions endpoint.
    """
    try:
        # This endpoint now uses the new modular structure
        # For now, we'll simulate a call to the new structure
        # In a real implementation, this would interface with the new services
        from src.chat.models import UserQuery
        from src.chat.services import ChatService
        from src.rag.services import RAGService
        import uuid
        from datetime import datetime

        # Create a user query from the legacy request
        user_query = UserQuery(
            id=str(uuid.uuid4()),
            content=request.message,
            selected_text=None,
            timestamp=datetime.now(),
            session_id=None,
            user_id=None
        )

        # Initialize services
        chat_service = ChatService()
        rag_service = RAGService()

        # Process the query using the new structure
        response = await chat_service.process_query(user_query, rag_service, temperature=0.7, session_id=None)

        return ChatResponse(response=response.content)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics Chatbot API"}


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "--serve":
        # Run the API server with the new modular app
        uvicorn.run("src.main:app", host="0.0.0.0", port=8000, reload=True)
    else:
        # Run the ingestion pipeline by default
        # This would be the original ingestion functionality
        # For now, we'll call the original function
        ingest_book()