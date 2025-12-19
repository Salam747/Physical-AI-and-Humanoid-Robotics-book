# Quickstart Guide: Free RAG Chatbot Backend

**Feature Branch**: `006-rag-backend`  
**Created**: 2025-12-09  
**Plan**: ../plan.md
**Specification**: ../spec.md

## Overview

This guide provides essential steps to quickly set up and run the Free RAG Chatbot Backend locally, and understand its basic functionality. This backend leverages FastAPI, Gemini API, Qdrant, and Neon Postgres to provide RAG capabilities for the "Physical AI Book".

## Prerequisites

Before you begin, ensure you have the following installed:

1.  **Python 3.11+**
2.  **pip** (Python package installer)
3.  **Git**
4.  **Docker Desktop** (for local Qdrant and Neon if not using cloud services)

## Step 1: Clone the Repository

First, clone the project repository to your local machine:

```bash
git clone https://github.com/your-username/Physical_AI_and_Humanoid_Robotics_book.git
cd Physical_AI_and_Humanoid_Robotics_book
git checkout 006-rag-backend
```

## Step 2: Set Up Environment Variables

The backend requires several environment variables for configuration. Create a file named `.env` in the `backend/` directory with the following content. **Replace the placeholder values with your actual credentials.**

```
# backend/.env
GOOGLE_API_KEY="YOUR_GEMINI_API_KEY"
QDRANT_URL="YOUR_QDRANT_CLOUD_URL"
QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
NEON_DB_URL="YOUR_NEON_POSTGRES_CONNECTION_STRING"

# Example local Qdrant/Neon (if running via Docker)
# QDRANT_URL="http://localhost:6333"
# NEON_DB_URL="postgresql://user:password@localhost:5432/dbname"
```

### Obtaining API Keys and URLs:

*   **Google Gemini API Key**:
    1.  Go to [Google AI Studio](https://aistudio.google.com/app/apikey).
    2.  Click "Get API key" and create a new API key.
    3.  Copy the generated key and paste it into `GOOGLE_API_KEY` in your `.env` file.
*   **Qdrant Cloud Account**:
    1.  Sign up for a free account at [Qdrant Cloud](https://cloud.qdrant.io/).
    2.  Create a new cluster.
    3.  Copy the "URL" and "API Key" for your cluster and paste them into `QDRANT_URL` and `QDRANT_API_KEY`.
*   **Neon Serverless Postgres**:
    1.  Sign up for a free account at [Neon](https://neon.tech/).
    2.  Create a new project and database.
    3.  Copy the connection string (usually in `postgres://...`) and paste it into `NEON_DB_URL`.

## Step 3: Install Dependencies

Navigate into the `backend/` directory and install the required Python packages:

```bash
cd backend/
pip install -r requirements.txt
```

## Step 4: Run Database Migrations (Neon)

*(This step assumes `alembic` will be used for migrations. Details will be provided in implementation.)*

```bash
# Example command - actual command will be in implementation phase
# alembic upgrade head
```

## Step 5: Ingest Book Content

Before you can query the chatbot, you need to ingest the book's content. Ensure your Markdown (`.mdx`) files are located in the `docs/` directory of the project root.

```bash
# From the backend/ directory
python -m ingest.ingest_book # This script will call the /ingest endpoint internally or run directly
```
Alternatively, once the server is running, you can call the `/ingest` API endpoint:

```bash
# Example using curl (replace with your actual server URL)
curl -X POST http://localhost:8000/ingest
```

## Step 6: Start the FastAPI Backend

From the `backend/` directory, start the FastAPI application:

```bash
uvicorn app:app --reload
```

The API will be available at `http://localhost:8000`.

## Step 7: Query the Chatbot

You can now send questions to the chatbot using the `/query` endpoint.

```bash
# Example using curl
curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d 
'{
           "question": "What is the main purpose of ROS 2?",
           "selected_text": null
         }'

# Example with selected_text override
curl -X POST http://localhost:8000/query \
     -H "Content-Type: application/json" \
     -d 
'{
           "question": "Explain this concept",
           "selected_text": "ROS 2 is a set of software libraries and tools that help you build robot applications."
         }'
```

## Step 8: Check Health Endpoint

```bash
curl http://localhost:8000/health
# Expected Output: {"status":"healthy"}
```

## Deployment (Vercel)

The project is configured for easy deployment to Vercel. Ensure your repository is linked to Vercel, and your environment variables are configured in the Vercel project settings. A `vercel.json` will be provided for auto-detection.
Pushing to your linked GitHub repository will trigger an automatic deployment.

```
