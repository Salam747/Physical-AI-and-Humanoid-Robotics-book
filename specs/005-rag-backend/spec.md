# Feature Specification: Free RAG Chatbot Backend

**Feature Branch**: `005-rag-backend`  
**Created**: 2025-12-09  
**Status**: Draft  
**Input**: User description: "/sp.specify — Detailed Spec for FREE RAG Chatbot Backend..."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Core RAG Query (Priority: P1)

As a user of the Physical AI book, I want to ask questions via an API and receive accurate answers generated from the book's content, so that I can quickly find information without reading through the whole book.

**Why this priority**: This is the core functionality of the chatbot. Without it, the feature has no value.

**Independent Test**: Can be fully tested by sending a POST request to the `/query` endpoint with a question. The returned answer should be relevant and factually correct based on the ingested book content.

**Acceptance Scenarios**:

1.  **Given** the book content is ingested, **When** a user sends a POST request to `/query` with `{"question": "What is ROS 2?"}`, **Then** the system returns a 200 OK response with a JSON body containing an accurate answer derived from the book's text.
2.  **Given** the book content is ingested, **When** a user asks a question whose answer is not in the book, **Then** the system returns an answer stating that the information is not available in the provided context.

---

### User Story 2 - Selected Text Query (Priority: P2)

As a user, I want to highlight a specific piece of text on the book's website and ask a question about *only that text*, so that I can get a highly focused explanation.

**Why this priority**: This provides a significant usability improvement, allowing for contextual "explain this" functionality, which is a common use case for documentation chatbots.

**Independent Test**: Can be tested by sending a POST request to `/query` with both a `question` and a `selected_text` field.

**Acceptance Scenarios**:

1.  **Given** a user provides `{"question": "Explain this in simple terms", "selected_text": "A digital twin is a virtual model of a physical object."}`, **Then** the system returns an answer explaining what a digital twin is, based ONLY on the provided sentence.
2.  **Given** the same request, **When** the question is about a topic not present in the `selected_text` (e.g., "What about ROS 2?"), **Then** the system returns a message indicating the answer is not in the provided text.

---

### User Story 3 - Automated Content Ingestion (Priority: P3)

As the book administrator, I want a script to process all `.mdx` files from the `/docs` directory, chunk them, generate embeddings, and load them into the vector DB (Qdrant) and metadata store (Neon).

**Why this priority**: This is crucial for the initial setup and for keeping the chatbot's knowledge base up-to-date with the book's content.

**Independent Test**: Can be tested by running the ingestion script and verifying that the document count in Qdrant and Neon matches the expected number of chunks from the source files.

**Acceptance Scenarios**:

1.  **Given** a directory with 10 `.mdx` files, **When** the ingestion script is run via the `/ingest` endpoint, **Then** the Qdrant collection for the book is populated with text chunks and their corresponding vectors, and the Neon database contains metadata for each chunk.

---

### Edge Cases

-   **What happens when `GEMINI_API_KEY` is missing or invalid?** The API MUST return a 401 Unauthorized or 500 Internal Server Error with a clear message: "GEMINI_API_KEY not found. Please add your free key to the .env file."
-   **How does the system handle Qdrant or Neon being unavailable?** The API should return a 503 Service Unavailable error with a message like "Database connection failed."
-   **What happens if a query returns no relevant documents?** The RAG pipeline should not proceed to the generation step. It should immediately return a message like "Sorry, I couldn't find any information in the book relevant to your question."
-   **How does the system handle API rate limits (15 RPM)?** The client-side code (in `utils/embeddings.py` or `rag/generator.py`) should implement a simple retry mechanism with exponential backoff or return a 429 Too Many Requests error.

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: System MUST provide a `POST /query` endpoint that accepts a JSON object with a `question` (string) and an optional `selected_text` (string).
-   **FR-002**: System MUST provide a `GET /health` endpoint that returns a 200 OK status.
-   **FR-003**: System MUST provide a `POST /ingest` endpoint that triggers the content ingestion process for a specified file path.
-   **FR-004**: The ingestion process MUST read all `.mdx` files from the book's `/docs` directory.
-   **FR-005**: The ingestion process MUST split text into chunks between 300 and 600 tokens.
-   **FR-006**: The system MUST use `gemini-1.5-flash` for generation and `models/text-embedding-004` for embeddings.
-   **FR-007**: The RAG retriever MUST fetch the top 5 chunks from Qdrant based on cosine similarity, with a minimum similarity score of 0.7.
-   **FR-008**: The entire codebase MUST NOT contain any `import openai` statements or rely on OpenAI APIs.
-   **FR-009**: The FastAPI application MUST have CORS enabled to allow requests from its deployed GitHub Pages domain.
-   **FR-010**: Every function in the codebase MUST have a docstring and an inline comment in both English and Urdu.

### Key Entities

-   **BookChunk**: A SQLAlchemy model representing a chunk of text from the book.
    -   `id`: Primary Key, Integer.
    -   `content`: The text of the chunk, Text.
    -   `source_file`: The originating file path (e.g., 'module1/01-ros2-architecture.mdx'), String.
    -   `vector_id`: The UUID of the corresponding vector in Qdrant, String.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: Ingestion script processes 100% of `.mdx` files in the target directory into Qdrant/Neon without any errors.
-   **SC-002**: Average response time for the `/query` endpoint is under 3 seconds when deployed on the Vercel free tier.
-   **SC-003**: When tested against a predefined list of 20 questions, the API provides a factually correct answer based on the book content for at least 18 out of 20 questions (90% accuracy).
-   **SC-004**: The project MUST be deployable to Vercel from a GitHub repository with a `vercel.json` configuration file and no manual steps.
