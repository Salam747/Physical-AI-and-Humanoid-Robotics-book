# Implementation Plan: Free RAG Chatbot Backend

**Branch**: `005-rag-backend` | **Date**: 2025-12-09 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/005-rag-backend/spec.md`

## Summary

This plan outlines the architecture for a 100% free-tier RAG (Retrieval-Augmented Generation) backend. The system will use a FastAPI server to expose an API for querying content from the "Physical AI and Humanoid Robotics" book. It will leverage Google's Gemini-1.5-Flash for answer generation, Gemini's `text-embedding-004` for creating vector embeddings, Qdrant Cloud for vector storage, and Neon for optional metadata storage. The entire stack is designed to be beginner-friendly and deployable on Vercel.

## Technical Context

**Language/Version**: Python 3.9+
**Primary Dependencies**: FastAPI, LangChain, Qdrant-Client, Google-GenerativeAI, SQLAlchemy
**Storage**: Qdrant Cloud (vectors), Neon Serverless Postgres (metadata)
**Testing**: Pytest
**Target Platform**: Vercel (or any serverless function provider)
**Project Type**: Web Application (Backend API)
**Performance Goals**: Average response time < 3 seconds on free tiers.
**Constraints**: Must operate within the free tier limits of all services (Gemini, Qdrant, Neon, Vercel). No paid APIs. No OpenAI dependencies.
**Scale/Scope**: Designed for a single book with moderate query load, suitable for a documentation website.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **[PASS] 100% FREE forever**: All chosen technologies (FastAPI, Qdrant Cloud, Neon, Gemini API, Vercel) have robust free tiers sufficient for this project's scope.
- **[PASS] Gemini API only**: The plan explicitly uses `google-generativeai` and `langchain-google-genai` with no mention of OpenAI or other providers.
- **[PASS] Beginner-friendly**: The architecture uses standard, well-documented libraries. The plan includes extensive commenting in the code.
- **[PASS] Spec-driven**: This plan is generated directly in response to a formal specification, adhering to the SDD workflow.

**Result**: All constitutional gates pass.

## Project Structure

### Documentation (this feature)

```text
specs/005-rag-backend/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── openapi.json
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by this command)
```

### Source Code (repository root)

The backend will be a self-contained application within the `backend/` directory.

```text
backend/
├── app.py                # FastAPI main server (routes: /query, /health, /ingest)
├── .env.example          # Environment variable template
├── README.md             # Setup and deployment guide
├── requirements.txt      # Python dependencies
├── db/
│   ├── __init__.py
│   └── models.py         # SQLAlchemy model for BookChunk
├── ingest/
│   └── ingest_data.py    # Script to read, chunk, and upload book content
├── rag/
│   ├── __init__.py
│   ├── generator.py      # Contains logic to call Gemini for answer generation
│   ├── pipeline.py       # Orchestrates the full RAG pipeline
│   └── retriever.py      # Contains logic to query Qdrant
├── tests/
│   ├── __init__.py
│   └── test_api.py       # Pytest tests for the /query endpoint
└── utils/
    ├── __init__.py
    └── embeddings.py     # Function to get embeddings from Gemini
```

**Structure Decision**: A single, self-contained `backend` directory is chosen. This structure is simple, portable, and aligns with the "web application" backend model. It clearly separates the chatbot's source code from the main project's documentation and specifications.

## Complexity Tracking

> No constitutional violations were detected. This section is not required.
