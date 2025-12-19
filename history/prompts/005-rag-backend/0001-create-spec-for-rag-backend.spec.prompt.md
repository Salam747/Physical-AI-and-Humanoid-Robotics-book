---
id: 1
title: Create Spec for RAG Backend
stage: spec
date: 2025-12-09
surface: agent
model: gemini-1.5-pro
feature: 005-rag-backend
branch: 005-rag-backend
user: Abdul Salam
command: /sp.specify
labels: ["backend", "rag", "api"]
links:
  spec: "specs/005-rag-backend/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "specs/005-rag-backend/spec.md"
  - "specs/005-rag-backend/checklists/requirements.md"
tests:
  - "none"
---

## Prompt

/sp.specify — Detailed Spec for FREE RAG Chatbot Backend## Backend TitleFree RAG Chatbot API (Gemini API + Qdrant + Neon + FastAPI) for Physical AI Book## Target Integration- Embed in Docusaurus book on GitHub Pages (via React component calling API)- Backend deployed to Vercel free tier (preferred) or Render- CORS enabled for GitHub Pages domain## Primary FocusBuild a lean FastAPI backend that:- Ingests book content (.md files from /docs) into Qdrant vectors using Gemini embeddings- Stores metadata (page IDs, etc.) in Neon Postgres- Handles RAG queries: Retrieve similar chunks from Qdrant, generate answer with Gemini 1.5 Flash- Supports "selected_text" override: If provided, use only that as context (ignore full book)## Success Criteria1. Ingest script loads 100% book content without error2. /query endpoint answers accurately (e.g., "What is ROS 2?" → correct from Module 1)3. Selected text works: Send {"question": "Explain this", "selected_text": "ROS 2 is middleware"} → answer only from that text4. Response time <3s on free tiers5. Deploys to Vercel with one GitHub push6. Beginner README: "Step 1: Get Gemini key from here [link]"## Mandatory Content- app.py: FastAPI app with async routes- rag/pipeline.py: LangChain chain – Qdrant retriever + Gemini generator- db/models.py: SQLAlchemy for Neon (table: book_chunks with id, text, metadata)- ingest/ingest_book.py: Script to read /docs/*.mdx, chunk text (500 chars), embed with Gemini, store in Qdrant + Neon- utils/embeddings.py: Function using google.generativeai.embed_content (free)- .env.example: GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DB_URL- requirements.txt: fastapi, uvicorn, sqlalchemy, psycopg2-binary, qdrant-client, google-generativeai, langchain-community, langchain-google-genai## Key Standards- Embeddings: Gemini text-embedding-004 (free)- RAG: Top-5 chunks from Qdrant (cosine similarity >0.7)- Error handling: If no Gemini key, return "Add your free key"- Tests: Pytest for /query (mock Gemini response)- Comments: Every function with Urdu + English explanation (e.g., # Yeh function book text ko chunks mein todta hai / This function chunks the book text)## Constraints- 100% free: No paid calls, handle rate limits (sleep if 15 RPM hit)- No OpenAI import- Chunk size: 300-600 tokens- Timeline: Generate all in one CLI run- Deployment: Vercel.json for config (auto-detect FastAPI)## Explicitly NOT Building- Frontend UI (do in Docusaurus later)- User auth (public API)- Paid features (e.g., no Gemini Pro if free limit hit)This spec is binding. Generate full backend now. /sp.specify he

## Response

Specification and quality checklist have been created:
- `specs/005-rag-backend/spec.md`
- `specs/005-rag-backend/checklists/requirements.md`

The specification is complete and has passed all quality checks. We are now ready to proceed to the planning phase with `/sp.plan`.
