# Data Model: ChatKit UI Integration for RAG System

## Entities

### UserQuery
**Description**: A text-based question or statement submitted through the ChatKit interface, which may reference general book content or specific selected text.

**Fields**:
- `id` (string, required): Unique identifier for the query
- `content` (string, required): The actual text of the user's query
- `selected_text` (string, optional): Specific text passage selected by the user (if any)
- `timestamp` (datetime, required): When the query was submitted
- `session_id` (string, optional): Identifier for the chat session (if applicable)
- `user_id` (string, optional): Identifier for the user (if authenticated)

**Validation rules**:
- `content` must be between 1 and 1000 characters
- `selected_text` must be between 1 and 5000 characters if provided
- `timestamp` must be in ISO 8601 format

### RetrievedContext
**Description**: Relevant segments of the Physical AI & Humanoid Robotics textbook that are retrieved by the RAG system to inform the response.

**Fields**:
- `id` (string, required): Unique identifier for the context chunk
- `content` (string, required): The text content from the textbook
- `source_document` (string, required): Reference to the original document
- `page_number` (integer, optional): Page number in the original document
- `section_title` (string, optional): Title of the section containing the content
- `similarity_score` (float, required): Score representing how relevant this context is to the query (0.0-1.0)
- `embedding_id` (string, required): ID of the vector embedding in Qdrant

**Validation rules**:
- `content` must be between 10 and 5000 characters
- `similarity_score` must be between 0.0 and 1.0
- `page_number` must be a positive integer if provided

### AIResponse
**Description**: The generated text response that addresses the user's query based solely on the retrieved textbook content.

**Fields**:
- `id` (string, required): Unique identifier for the response
- `content` (string, required): The text content of the AI's response
- `query_id` (string, required): Reference to the original user query
- `retrieved_context_ids` (array of strings, required): References to the context chunks used
- `timestamp` (datetime, required): When the response was generated
- `confidence_score` (float, required): Confidence level in the response accuracy (0.0-1.0)
- `source_documents` (array of strings, required): List of documents referenced in the response

**Validation rules**:
- `content` must be between 10 and 10000 characters
- `confidence_score` must be between 0.0 and 1.0
- `timestamp` must be in ISO 8601 format

### ChatSession
**Description**: A sequence of related queries and responses that maintains conversational context while remaining grounded in the textbook content.

**Fields**:
- `id` (string, required): Unique identifier for the chat session
- `created_at` (datetime, required): When the session was created
- `last_interaction` (datetime, required): When the last query/response occurred
- `user_id` (string, optional): Identifier for the user (if authenticated)
- `is_active` (boolean, required): Whether the session is currently active
- `query_count` (integer, required): Number of queries in this session

**Validation rules**:
- `created_at` and `last_interaction` must be in ISO 8601 format
- `query_count` must be a non-negative integer

## Relationships

- `UserQuery` (1) → (0..n) `RetrievedContext`: A query may result in multiple context chunks being retrieved
- `UserQuery` (1) → (1) `AIResponse`: Each query generates one response
- `AIResponse` (1) → (1..n) `RetrievedContext`: A response is based on one or more context chunks
- `ChatSession` (1) → (0..n) `UserQuery`: A session contains multiple queries
- `ChatSession` (1) → (0..n) `AIResponse`: A session contains multiple responses

## State Transitions

### ChatSession
- `created` → `active`: When the first query is made in the session
- `active` → `inactive`: When the session has been idle for more than 30 minutes
- `inactive` → `archived`: When the session data is moved to long-term storage