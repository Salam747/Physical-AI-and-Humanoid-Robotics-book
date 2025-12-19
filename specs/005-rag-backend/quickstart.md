# Quickstart: RAG Backend

This document provides the essential steps to get the RAG backend running locally. For more detailed explanations, see the `README.md` file inside the `backend/` directory.

## 1. Prerequisites

-   Python 3.9+
-   Access to Google AI Studio (for Gemini API Key)
-   A Qdrant Cloud account (free tier)
-   A Neon account (free tier)

## 2. Environment Setup

```bash
# 1. Navigate to the backend directory
cd backend

# 2. Create and activate a virtual environment
python -m venv venv
source venv/bin/activate # or venv\Scripts\activate on Windows

# 3. Install dependencies
pip install -r requirements.txt

# 4. Configure environment variables
cp .env.example .env
# Edit .env and add your keys from Gemini, Qdrant, and Neon.
```

## 3. Run the Application

There are two main processes: running the ingestion script (one-time setup) and running the API server.

### A. Run the API Server

```bash
# From the 'backend' directory
uvicorn app:app --reload
```

The API will be available at `http://127.0.0.1:8000`. You can access the interactive Swagger UI at `http://127.0.0.1:8000/docs`.

### B. Ingest Content

Use a tool like `curl` or the Swagger UI to send a POST request to the `/ingest` endpoint.

```bash
curl -X POST http://127.0.0.1:8000/ingest \
-H "Content-Type: application/json" \
-d '{"file_path": "../path/to/your/book.md"}'
```

## 4. Query the API

Send a POST request to the `/query` endpoint.

```bash
curl -X POST http://127.0.0.1:8000/query \
-H "Content-Type: application/json" \
-d '{"question": "What is a digital twin?"}'
```

