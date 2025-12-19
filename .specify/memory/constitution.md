# Project Constitution — FREE RAG Chatbot using Gemini API only

## Project Title
Free-Tier RAG Chatbot for Physical AI Book (Gemini 1.5 Flash + Qdrant + Neon + FastAPI)

## Core Principles
- 100% FREE forever: Zero paid APIs, zero paid hosting
- Gemini API only: No OpenAI SDK, no OpenAI embeddings, no OpenAI anything
- Beginner-friendly: Every file fully commented, zero advanced tricks
- Spec-driven: Gemini CLI generates entire backend automatically
- Works with selected text + full book search

## Key Standards & Tools (all free)
- LLM & Embeddings → Google Gemini 1.5 Flash API (free tier: 15 RPM + 1M tokens/day)
- Backend → FastAPI + Uvicorn
- Vector DB → Qdrant Cloud Free Tier (1 GB, 100k vectors)
- Metadata DB → Neon Serverless Postgres (free 0.5 GB)
- Embedding model → text-embedding-004 (Gemini free) or sentence-transformers/all-MiniLM-L6-v2 (fully local fallback)
- RAG framework → LangChain Community (free) with GoogleGenerativeAI integration
- Hosting → Render.com free tier or Vercel (auto-deploy from GitHub)
- Frontend embed → Simple React component in Docusaurus (later phase)

## Mandatory Folders Gemini Must Create
```
/backend
├── app.py                → FastAPI main server
├── rag/
│   ├── retriever.py      → Qdrant search
│   ├── generator.py      → Gemini 1.5 Flash call
│   └── pipeline.py       → Full RAG chain
├── db/                   → Neon + SQLAlchemy models
├── ingest/               → Script to load book .txt/.md into Qdrant
├── utils/
│   └── embeddings.py     → Gemini embedding function
├── .env.example
├── requirements.txt
└── README.md             → Full beginner setup guide
```

## API Endpoints (must exist)
- `POST /query`          → `{ "question": "...", "selected_text": "optional..." }`
- `GET  /health`
- `POST /ingest`         → Load book content (run once)

## Constraints
- No OpenAI import anywhere
- Must work with free Gemini API key (gemini.google.com → API key)
- Total cost = ₹0
- Response time < 3 seconds
- Supports selected text override
- Code < 1500 lines total

## Success Criteria
- User asks question → correct answer from book (tested on 20 sample Qs)
- User selects text → answer only from that text (no full book hallucination)
- Runs locally: `uvicorn app:app --reload`
- Deploys free on Render/Vercel
- README has exact steps for beginner (even copy-paste .env)

## Tone & Voice
"Big brother teaching younger brother"
Har line comment mein Urdu + English explanation ho

## Approval
This constitution is binding. Gemini must refuse any suggestion of OpenAI, Anthropic, or paid APIs.