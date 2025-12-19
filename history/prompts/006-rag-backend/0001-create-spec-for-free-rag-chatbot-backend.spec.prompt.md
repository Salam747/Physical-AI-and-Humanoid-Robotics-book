---
id: 0001
title: Create Spec for Free RAG Chatbot Backend
stage: spec
date: 2025-12-09
surface: agent
model: gemini-pro
feature: 006-rag-backend
branch: 006-rag-backend
user: Abdul Salam
command: /sp.specify Free RAG Chatbot API (Gemini API + Qdrant + Neon + FastAPI) for Physical AI Book ... [full feature description]
labels: ["spec-creation", "rag-backend"]
links:
  spec: specs/006-rag-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-rag-backend/spec.md
  - specs/006-rag-backend/checklists/requirements.md
tests:
  - "none"
---

## Prompt

/sp.specify Free RAG Chatbot API (Gemini API + Qdrant + Neon + FastAPI) for Physical AI Book (Gemini 1.5 Flash + Qdrant + Neon + FastAPI). Build a lean FastAPI backend that: Ingests book content (.md files from /docs) into Qdrant vectors using Gemini embeddings; Stores metadata (page IDs, etc.) in Neon Postgres; Handles RAG queries: Retrieve similar chunks from Qdrant, generate answer with Gemini 1.5 Flash; Supports 'selected_text' override: If provided, use only that as context (ignore full book). Target Integration: Embed in Docusaurus book on GitHub Pages (via React component calling API). Backend deployed to Vercel free tier (preferred) or Render. CORS enabled for GitHub Pages domain. Success Criteria: Ingest script loads 100% book content without error; /query endpoint answers accurately (e.g., 'What is ROS 2?' -> correct from Module 1); Selected text works; Response time <3s on free tiers; Deploys to Vercel with one GitHub push; Beginner README. Mandatory Content: app.py, rag/pipeline.py, db/models.py, ingest/ingest_book.py, utils/embeddings.py, .env.example, requirements.txt, README.md. Key Standards: Embeddings: Gemini text-embedding-004; RAG: Top-5 chunks from Qdrant (cosine similarity >0.7); Error handling: If no Gemini key, return 'Add your free key'; Tests: Pytest for /query (mock Gemini response); Comments: Every function with Urdu + English explanation. Constraints: 100% free; No OpenAI import; Chunk size: 300-600 tokens; Timeline: Generate all in one CLI run; Deployment: Vercel.json. Explicitly NOT Building: Frontend UI, User auth, Paid features. This spec is binding. Generate full backend now.

## Response snapshot

The feature specification for the `Free RAG Chatbot Backend` has been successfully created and validated.
*   **Branch Name**: `006-rag-backend`
*   **Spec File Path**: `specs/006-rag-backend/spec.md`
*   **Checklist Results**: The `specs/006-rag-backend/checklists/requirements.md` file has been updated. The specification is considered high quality and complete within the project's context, with intentional deviations from generic guidelines noted in the checklist. No further clarifications are needed at this stage.

This feature is now ready for the planning phase.
**Next Action**: You can now proceed to create the plan for this feature by running `/sp.plan`.

## Outcome

- ✅ Impact: Successfully created a comprehensive and validated feature specification for the RAG Chatbot Backend.
- 🧪 Tests: None (spec creation is not test-driven, but spec includes testing criteria).
- 📁 Files: Created `specs/006-rag-backend/spec.md` and `specs/006-rag-backend/checklists/requirements.md`.
- 🔁 Next prompts: `/sp.plan`
- 🧠 Reflection: The agent successfully parsed a detailed user description and generated a structured spec, handling various sections and validation according to instructions. Intentional deviations from generic spec guidelines were documented.

## Evaluation notes (flywheel)

- Failure modes observed: `create-new-feature.ps1` failed after branch creation, requiring manual path acquisition, and `create-phr.ps1` script not found, requiring manual PHR creation.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Investigate `create-new-feature.ps1` and `create-phr.ps1` script issues.
