"""
RAG (Retrieval Augmented Generation) business logic services.
Based on the data-model.md specification and user stories.
"""
from typing import List, Tuple
import uuid
from datetime import datetime

from ..chat.models import UserQuery, RetrievedContext, AIResponse
from ..core.config import settings
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere


class RAGService:
    """
    Service class to handle RAG business logic.
    """
    
    def __init__(self):
        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True
        )

        # Initialize Cohere client
        self.cohere_client = cohere.Client(settings.COHERE_API_KEY)

        # Set the collection name for textbook content
        self.collection_name = settings.TEXTBOOK_COLLECTION_NAME
    
    async def retrieve_context(self, query: str, selected_text: str = None) -> List[RetrievedContext]:
        """
        Retrieve relevant context from the textbook based on the query.
        """
        # Prepare the query text (use selected text if provided, otherwise use the query)
        search_text = selected_text if selected_text else query

        # Generate embedding for the search text using Cohere
        embedding_response = self.cohere_client.embed(
            texts=[search_text],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        query_embedding = embedding_response.embeddings[0]

        # Search in Qdrant for similar content
        search_results = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=settings.SEARCH_LIMIT,
            score_threshold=settings.MIN_SIMILARITY_SCORE
        )

        # Convert search results to RetrievedContext models
        retrieved_contexts = []
        for result in search_results.points:  # Access the points attribute
            payload = result.payload
            retrieved_context = RetrievedContext(
                id=str(result.id),
                content=payload.get("content", ""),
                source_document=payload.get("source_document", ""),
                page_number=payload.get("page_number"),
                section_title=payload.get("section_title"),
                similarity_score=result.score,
                embedding_id=str(result.id)
            )
            retrieved_contexts.append(retrieved_context)

        return retrieved_contexts
    
    async def generate_response(
        self,
        query: str,
        retrieved_contexts: List[RetrievedContext],
        temperature: float = 0.7
    ) -> str:
        """
        Generate a response based on the query and retrieved contexts.
        """
        # Prepare the context content
        context_content = "\n\n".join([ctx.content for ctx in retrieved_contexts])

        # Construct the prompt to ground the response in textbook content
        prompt = f"""
        You are an AI assistant for the Physical AI & Humanoid Robotics textbook.
        Your responses must be based solely on the provided textbook content.
        Do not fabricate or hallucinate information.
        If the provided context doesn't contain information to answer the query,
        state that you don't have enough information from the textbook to answer.

        Textbook content:\n{context_content}

        Question: {query}

        Answer:
        """

        # Generate response using Cohere Chat API
        response = self.cohere_client.chat(
            model=settings.COHERE_MODEL,
            message=query,
            documents=[{"title": f"Document {i}", "snippet": ctx.content} for i, ctx in enumerate(retrieved_contexts, 1)],
            temperature=temperature,
        )

        return response.text
    
    async def validate_query(self, query: str, selected_text: str = None) -> Tuple[bool, float, List[str]]:
        """
        Validate if a query can be answered using the available textbook content.
        Returns: (is_valid, confidence, relevant_sources)
        """
        # Retrieve context for the query
        contexts = await self.retrieve_context(query, selected_text)

        if not contexts:
            return False, 0.0, []

        # Calculate average similarity score as confidence
        avg_similarity = sum(ctx.similarity_score for ctx in contexts) / len(contexts)

        # Get unique source documents
        source_documents = list(set(ctx.source_document for ctx in contexts))

        # Consider valid if at least one context has good similarity score
        is_valid = any(ctx.similarity_score >= settings.MIN_SIMILARITY_SCORE for ctx in contexts)

        return is_valid, avg_similarity, source_documents

    async def verify_response_grounding(
        self,
        query: str,
        response: str,
        retrieved_contexts: List[RetrievedContext]
    ) -> Tuple[bool, float]:
        """
        Verify that a response is properly grounded in the retrieved contexts.
        Returns: (is_grounded, confidence_score)
        """
        if not retrieved_contexts:
            return False, 0.0

        # Calculate how well the response is supported by the contexts
        response_words = set(response.lower().split())
        total_context_support = 0.0
        total_weight = 0.0

        for ctx in retrieved_contexts:
            context_words = set(ctx.content.lower().split())

            # Calculate overlap between response and context
            overlap = len(response_words.intersection(context_words))
            max_possible_overlap = min(len(response_words), len(context_words))

            if max_possible_overlap > 0:
                overlap_ratio = overlap / max_possible_overlap
                # Weight the support by the context's similarity score
                context_support = overlap_ratio * ctx.similarity_score
                total_context_support += context_support
                total_weight += ctx.similarity_score

        # Calculate weighted average support
        if total_weight > 0:
            grounding_score = total_context_support / total_weight
        else:
            grounding_score = 0.0

        # Determine if the response is sufficiently grounded
        is_grounded = grounding_score >= settings.MIN_SIMILARITY_SCORE

        return is_grounded, grounding_score


# Dependency for FastAPI
def get_rag_service():
    return RAGService()