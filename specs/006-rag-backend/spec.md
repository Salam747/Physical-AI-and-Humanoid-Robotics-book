# Feature Specification: Free RAG Chatbot Backend for Physical AI Book

**Feature Branch**: `006-rag-backend`  
**Created**: 2025-12-09  
**Status**: Draft  
**Input**: User description: "Free RAG Chatbot API (Gemini API + Qdrant + Neon + FastAPI) for Physical AI Book (Gemini 1.5 Flash + Qdrant + Neon + FastAPI). Build a lean FastAPI backend that: Ingests book content (.md files from /docs) into Qdrant vectors using Gemini embeddings; Stores metadata (page IDs, etc.) in Neon Postgres; Handles RAG queries: Retrieve similar chunks from Qdrant, generate answer with Gemini 1.5 Flash; Supports 'selected_text' override: If provided, use only that as context (ignore full book). Target Integration: Embed in Docusaurus book on GitHub Pages (via React component calling API). Backend deployed to Vercel free tier (preferred) or Render. CORS enabled for GitHub Pages domain. Success Criteria: Ingest script loads 100% book content without error; /query endpoint answers accurately (e.g., 'What is ROS 2?' -> correct from Module 1); Selected text works; Response time <3s on free tiers; Deploys to Vercel with one GitHub push; Beginner README. Mandatory Content: app.py, rag/pipeline.py, db/models.py, ingest/ingest_book.py, utils/embeddings.py, .env.example, requirements.txt, README.md. Key Standards: Embeddings: Gemini text-embedding-004; RAG: Top-5 chunks from Qdrant (cosine similarity >0.7); Error handling: If no Gemini key, return 'Add your free key'; Tests: Pytest for /query (mock Gemini response); Comments: Every function with Urdu + English explanation. Constraints: 100% free; No OpenAI import; Chunk size: 300-600 tokens; Timeline: Generate all in one CLI run; Deployment: Vercel.json. Explicitly NOT Building: Frontend UI, User auth, Paid features. This spec is binding. Generate full backend now."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Ingest Book Content (Priority: P1)

As a user, I want to ingest the book's content (Markdown files from `/docs`) into the RAG system so that the chatbot can answer questions based on the book.

**Why this priority**: This is an essential foundational step for the RAG system to function. Without ingested content, the chatbot cannot provide answers.

**Independent Test**: This can be fully tested by running the ingest script and verifying that the book content is correctly stored in Qdrant vectors and associated metadata in Neon Postgres.

**Acceptance Scenarios**:

1.  **Given** a set of `.md` files residing in the `/docs` directory, **When** the ingest script is successfully executed, **Then** 100% of the book content is accurately loaded into Qdrant vectors and its corresponding metadata into Neon Postgres without any errors.

---

### User Story 2 - Query the Chatbot for Book Content (Priority: P1)

As a user, I want to ask natural language questions about the "Physical AI Book" and receive accurate, concise answers that are solely derived from the book's content.

**Why this priority**: This represents the core functionality and primary value proposition of the RAG chatbot.

**Independent Test**: This can be fully tested by sending a series of diverse questions to the `/query` endpoint and evaluating the relevance and accuracy of the generated answers against the book's content.

**Acceptance Scenarios**:

1.  **Given** the book content has been successfully ingested, **When** a user sends a question to the `POST /query` API endpoint, **Then** the system returns an accurate answer derived strictly from the ingested book content.
2.  **Given** a user attempts to query the system, **When** the system detects a missing or invalid Gemini API key, **Then** the system returns an informative error message instructing the user to configure their free API key.

---

### User Story 3 - Query with Selected Text Override (Priority: P2)

As a user, I want to provide a specific piece of text (e.g., from a highlighted section) along with my question, so that the chatbot's answer is strictly based on the provided text, and it ignores the wider book content for that particular query.

**Why this priority**: This enables focused, context-aware question answering, prevents broad search results, and helps avoid potential hallucinations by limiting the RAG scope.

**Independent Test**: This can be fully tested by providing a question and specific `selected_text` to the `/query` endpoint and then verifying that the generated answer is entirely contained within, and logically consistent with, only the provided `selected_text`.

**Acceptance Scenarios**:

1.  **Given** the book content has been ingested, **When** a user sends a question to the `POST /query` endpoint along with a `selected_text` parameter, **Then** the system generates an answer using only the `selected_text` as its contextual basis, completely disregarding other parts of the ingested book content.

---

### Edge Cases

-   What happens when the Gemini API key is missing or invalid? (Covered in FRs and Acceptance Scenarios).
-   How does the system handle Gemini API rate limits (15 RPM hit)? (System should implement a retry mechanism or a graceful degradation message).
-   What if a query yields no relevant chunks from Qdrant (e.g., cosine similarity below 0.7)? (System should return a polite "No relevant information found" or similar message).
-   What if the input `selected_text` is empty or too short to be meaningful? (System should fall back to full book RAG or return an error/warning).

## Requirements *(mandatory)*

### Functional Requirements

-   **FR-001**: The system MUST provide a FastAPI backend accessible via HTTP.
-   **FR-002**: The system MUST ingest book content from Markdown (`.mdx`) files located in the `/docs` directory of the Docusaurus project.
-   **FR-003**: The system MUST chunk ingested text content into segments between 300-600 tokens.
-   **FR-004**: The system MUST embed text chunks using the Gemini `text-embedding-004` model (free tier).
-   **FR-005**: The system MUST store embedded text chunks in Qdrant (free tier).
-   **FR-006**: The system MUST store associated metadata (e.g., source file path, page ID, chunk ID) for each text chunk in Neon Serverless Postgres (free tier).
-   **FR-007**: The system MUST implement a RAG pipeline that retrieves a maximum of 5 relevant chunks from Qdrant based on cosine similarity, with a threshold of >0.7.
-   **FR-008**: The system MUST generate answers using the Gemini 1.5 Flash model (free tier).
-   **FR-009**: The system MUST expose a `POST /query` API endpoint that accepts a JSON payload `{"question": "string", "selected_text": "string (optional)"}`.
-   **FR-010**: The system MUST expose a `GET /health` API endpoint that returns a 200 OK status.
-   **FR-011**: The system MUST expose a `POST /ingest` API endpoint to trigger the loading of book content from `/docs` into Qdrant and Neon.
-   **FR-012**: The system MUST use only the `selected_text` as the context for answer generation if it is provided in the `POST /query` request, overriding the full book content RAG.
-   **FR-013**: The backend project MUST include a `README.md` file with clear, beginner-friendly setup instructions, including how to obtain a Gemini API key.
-   **FR-014**: All Python functions and significant code blocks MUST include comments with both Urdu and English explanations.
-   **FR-015**: The backend MUST have a file structure as defined in the project constitution: `app.py`, `rag/pipeline.py`, `db/models.py`, `ingest/ingest_book.py`, `utils/embeddings.py`, `.env.example`, `requirements.txt`.

### Key Entities *(include if feature involves data)*

-   **BookChunk**: Represents a segmented portion of the book's text.
    *   **Attributes**: `id` (unique identifier), `text_content` (the actual text segment), `embedding` (vector representation), `metadata` (JSON object containing `source_file`, `page_number`, etc.).
-   **Question**: The natural language input provided by the user for which an answer is sought.
-   **Answer**: The generated natural language response from the RAG pipeline, based on relevant `BookChunk`s.

## Success Criteria *(mandatory)*

### Measurable Outcomes

-   **SC-001**: The ingest script successfully loads 100% of all book content (`.mdx` files in `/docs`) into Qdrant and Neon without encountering any errors or data loss.
-   **SC-002**: The `POST /query` endpoint provides accurate answers for at least 85% of a predefined set of 20 sample questions, as verified by human evaluation.
-   **SC-003**: When a `selected_text` is provided with a question to the `POST /query` endpoint, the generated answer is strictly and demonstrably derived *only* from the `selected_text` provided, for 100% of tested cases.
-   **SC-004**: The `POST /query` endpoint consistently returns a response within 3 seconds for 95% of requests when deployed on free-tier Vercel/Render.
-   **SC-005**: The backend project successfully deploys to Vercel (or Render) using a single GitHub push, with no manual configuration steps required post-deployment beyond environment variables.
-   **SC-006**: The `README.md` file enables a beginner to successfully set up, run, and query the backend locally, and deploy it to Vercel, by following the provided instructions within 30 minutes.

### Non-Functional Requirements

-   **NFR-001 (Cost)**: The entire backend system (including LLM, vector DB, metadata DB, hosting) MUST operate at a total cost of ₹0.
-   **NFR-002 (Interoperability)**: The codebase MUST NOT contain any imports or references to OpenAI, Anthropic, or other paid third-party LLM APIs.
-   **NFR-003 (Resilience)**: The system MUST gracefully handle Gemini API rate limits (15 RPM) by implementing a robust retry mechanism or appropriate back-off strategy to prevent service interruptions.
-   **NFR-004 (Performance)**: The `POST /query` endpoint MUST respond to requests within 3 seconds for 95% of queries on free-tier infrastructure.
-   **NFR-005 (Deployment)**: The backend MUST be deployable to Vercel (preferred) or Render free tiers.
-   **NFR-006 (Security)**: The API MUST have CORS (Cross-Origin Resource Sharing) enabled specifically for the GitHub Pages domain where the Docusaurus frontend will be hosted.

## Assumptions

-   The "Physical AI Book" content will be available as Markdown (`.mdx`) files within a `/docs` directory, directly accessible by the ingest script.
-   The definition of "accurate answer" (SC-002) will be based on a human judgment of relevance and correctness relative to the book's content.
-   The RAG strategy employing top-5 chunks with cosine similarity >0.7 will be sufficient to generate high-quality and relevant answers.
-   Necessary environment variables (`GOOGLE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`, `NEON_DB_URL`) will be correctly provided by the user in an `.env` file for local development and configured as environment secrets for deployment.
-   The `python-dotenv` library will be used to load environment variables.

## Constraints

-   **Platform**: Exclusively Google Gemini API (1.5 Flash for generation, `text-embedding-004` for embeddings).
-   **Cost**: Absolutely zero recurring costs for all components.
-   **OpenAI Exclusion**: No OpenAI SDK, API, or services are permitted.
-   **Text Chunking**: Text chunks MUST be between 300 and 600 tokens in length.
-   **Timeline**: The entire backend development from this spec should be achievable within a single CLI run for full generation.
-   **Deployment**: Vercel `vercel.json` configuration for auto-detection and deployment of FastAPI.

## Explicitly NOT Building

-   **Frontend User Interface (UI)**: This specification does not cover any UI development. The integration with a Docusaurus React component is a separate frontend concern for a later phase.
-   **User Authentication/Authorization**: The API will be publicly accessible, and no user authentication or authorization mechanisms will be implemented.
-   **Paid Features**: The backend will not utilize any paid features of Gemini, Qdrant, Neon, or hosting providers, even if free-tier limits are reached. The system should gracefully handle limitations.
-   **Advanced Monitoring/Alerting**: Basic health checks will be implemented, but comprehensive monitoring and alerting infrastructure is out of scope.