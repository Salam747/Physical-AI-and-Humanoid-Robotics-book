# Implementation Plan: Free RAG Chatbot Backend

**Branch**: `006-rag-backend` | **Date**: 2025-12-09 | **Spec**: ../spec.md
**Input**: Feature specification from `/specs/006-rag-backend/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This plan outlines the implementation of a Free RAG Chatbot Backend for the "Physical AI Book." The primary requirement is to build a lean FastAPI backend capable of ingesting book content, storing it in Qdrant and Neon Postgres, and handling RAG queries using Gemini 1.5 Flash. It will also support a "selected_text" override for contextual answering. The technical approach adheres strictly to free-tier services and the Gemini API ecosystem, ensuring zero operational cost as mandated by the project constitution.

## Technical Context

**Language/Version**: Python 3.11  
**Primary Dependencies**: FastAPI, Uvicorn, Qdrant Client, google-generativeai (for Gemini API), LangChain Community, SQLAlchemy, psycopg2-binary, python-dotenv  
**Storage**: Qdrant (vector database for text embeddings), Neon Serverless Postgres (relational database for chunk metadata)  
**Testing**: Pytest  
**Target Platform**: Linux server (compatible with Vercel/Render free-tier hosting environments)  
**Project Type**: Backend API (serving a future Docusaurus React frontend)  
**Performance Goals**: `POST /query` endpoint to respond within 3 seconds (95th percentile)  
**Constraints**: 100% free-tier services, exclusive use of Gemini API, no OpenAI components, text chunk sizes between 300-600 tokens, deployment via Vercel `vercel.json` configuration.  
**Scale/Scope**: Designed for a single book's content, serving a single Docusaurus frontend application.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The plan adheres to all core principles and mandatory requirements outlined in the project's constitution (`.specify/memory/constitution.md`).

**Core Principles:**
-   **100% FREE forever**: PASSED. All selected technologies (Gemini API free tier, Qdrant Cloud Free Tier, Neon Serverless Postgres free tier, Vercel/Render free tier) are explicitly free and align with the `₹0 cost` mandate.
-   **Gemini API only**: PASSED. The plan exclusively specifies Google Gemini 1.5 Flash for LLM and `text-embedding-004` for embeddings, with explicit prohibition of OpenAI components.
-   **Beginner-friendly**: PASSED. The spec mandates Urdu + English comments and a beginner-friendly `README.md`, which will be enforced in implementation.
-   **Spec-driven**: PASSED. This plan is directly derived from the `/sp.specify` output.
-   **Works with selected text + full book search**: PASSED. The spec explicitly includes a user story and functional requirement for `selected_text` override.

**Mandatory Folders & API Endpoints:**
-   The plan's proposed `Project Structure` matches the mandatory folders.
-   The `Functional Requirements` in `spec.md` confirm the existence of `POST /query`, `GET /health`, and `POST /ingest` API endpoints. PASSED.

**Constraints (from Constitution):**
-   `No OpenAI import anywhere`: PASSED. Explicitly stated in spec and plan.
-   `Must work with free Gemini API key`: PASSED. Explicitly stated.
-   `Total cost = ₹0`: PASSED. Explicitly stated.
-   `Response time < 3 seconds`: PASSED. Performance goal included.
-   `Supports selected text override`: PASSED. User story and FR included.
-   `Code < 1500 lines total`: This is an implementation detail that will be monitored during development, but is a goal. PASSED (as a guiding principle).

All constitutional gates are met and justified by the chosen technologies and approach.

## Project Structure

### Documentation (this feature)

```text
specs/006-rag-backend/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── app.py                # FastAPI main server
├── rag/
│   ├── retriever.py      # Qdrant search logic
│   ├── generator.py      # Gemini 1.5 Flash call logic
│   └── pipeline.py       # Full RAG chain orchestration
├── db/
│   └── models.py         # SQLAlchemy models for Neon Postgres
├── ingest/
│   └── ingest_book.py    # Script to load book content into Qdrant & Neon
├── utils/
│   └── embeddings.py     # Gemini embedding function
├── .env.example          # Environment variable example
├── requirements.txt      # Python dependencies
└── README.md             # Beginner setup guide
```

**Structure Decision**: The source code structure is adopted from the project constitution's "Mandatory Folders Gemini Must Create" section, which aligns with a single backend service (`Option 2` in template terms) tailored for this project's needs.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*(No constitution violations were detected that require justification, as the design adheres strictly to the defined principles and constraints.)*

## Phase 0: Outline & Research

### Research Findings (research.md)

No critical research questions requiring external investigation were identified at this stage. The technologies and approaches are well-defined in the `spec.md` and `constitution.md`.
Specific implementation details regarding LangChain integration patterns for Gemini and Qdrant, FastAPI setup, and Neon/SQLAlchemy usage are standard and will be handled during the implementation phase.

*(A placeholder `research.md` file will be generated to indicate completion of this phase.)*

## Phase 1: Design & Contracts

### Data Model (data-model.md)

Based on the "Key Entities" in `spec.md`, the primary data model is:

-   **BookChunk**: Represents a segmented portion of the book's text.
    *   **Attributes**:
        *   `id` (UUID or integer, primary key): Unique identifier for the chunk.
        *   `text_content` (TEXT): The actual text segment (300-600 tokens).
        *   `embedding` (VECTOR): Vector representation generated by Gemini `text-embedding-004`.
        *   `metadata` (JSONB/JSON): Stores additional information like `source_file` (e.g., `module1-ros2.mdx`), `page_number` (INTEGER), `chunk_index` (INTEGER, for ordering within a source file).

### API Contracts (contracts/)

API contracts will be defined using OpenAPI (YAML) for the following endpoints:

-   **POST /query**
    *   **Description**: Submits a natural language question about the book, with an optional `selected_text` override, and receives an AI-generated answer.
    *   **Request Body**:
        ```json
        {
          "question": "string",
          "selected_text": "string" (optional)
        }
        ```
    *   **Responses**:
        *   `200 OK`:
            ```json
            {
              "answer": "string",
              "source_chunks": [
                {
                  "id": "string",
                  "text_content": "string",
                  "metadata": { /* ... */ }
                }
              ] (optional)
            }
            ```
        *   `400 Bad Request`: If input is invalid.
        *   `500 Internal Server Error`: For issues like missing API key, rate limits.

-   **GET /health**
    *   **Description**: Checks the health and availability of the API service.
    *   **Responses**:
        *   `200 OK`:
            ```json
            {
              "status": "string" // e.g., "healthy"
            }
            ```

-   **POST /ingest**
    *   **Description**: Triggers the ingestion process of book content from `/docs` into Qdrant and Neon.
    *   **Request Body**: `None` (or an optional `{"force_reingest": boolean}` to clear existing data).
    *   **Responses**:
        *   `202 Accepted`: Ingestion started.
        *   `200 OK`: If ingestion completes synchronously (unlikely for large books).
        *   `500 Internal Server Error`: If ingestion fails.

*(Specific OpenAPI YAML files will be generated in the `contracts/` directory.)*

### Quickstart Guide (quickstart.md)

A `quickstart.md` will be generated providing essential steps for a beginner to get the backend running locally, including environment setup and basic usage instructions.

*(The `quickstart.md` file will be generated.)*