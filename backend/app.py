# Murad... Asking for brother's help...
# This is the main server file using FastAPI. It creates the API endpoints.
# Yeh FastAPI ka istemal karke main server file hai. Yeh API endpoints banati hai.

# Endpoints (API ke raaste):
# - GET /health: Checks if the server is running. (Server chal raha hai ya nahi, yeh check karta hai).
# - POST /query: The main endpoint to ask questions. (Sawal puchne ke liye main endpoint).
# - POST /ingest: To load the book content into the database (run only once). (Kitab ka content database mein load karne ke liye - sirf ek baar chalana hai).

import os
from fastapi import FastAPI, HTTPException, Depends, Header
from fastapi.middleware.cors import CORSMiddleware # Import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
from sqlalchemy.orm import Session
from typing import Optional

# Import the main RAG pipeline function
# Main RAG pipeline function import karo
from rag.pipeline import query_rag_pipeline

# Import the ingestion script function (we will create this file next)
# Ingestion script function import karo (hum yeh file iske baad banayenge)
from ingest.ingest_data import ingest_data

# Import authentication router
# Authentication router import karo
from auth.routes import router as auth_router

# Import chat history router
# Chat history router import karo
from chat.routes import router as chat_router

# Import database dependencies
# Database dependencies import karo
from db.models import get_db

# Load environment variables from .env file
# .env file se environment variables load karo
load_dotenv()

# --- FastAPI App Initialization ---
app = FastAPI(
    title="Physical AI Book RAG API",
    description="An API for querying the 'Physical AI and Humanoid Robotics' book using Gemini.",
    version="1.0.0",
)

# Configure CORS (Cross-Origin Resource Sharing)
# CORS (Cross-Origin Resource Sharing) configure karein
# Get allowed origins from environment or use defaults
allowed_origins = os.getenv("ALLOWED_ORIGINS", "").split(",")
origins = [
    "https://salam747.github.io",  # GitHub Pages domain
    "http://localhost",
    "http://localhost:3000",  # Default for many frontend development servers
    "http://localhost:3001",  # Docusaurus default port
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
] + [origin.strip() for origin in allowed_origins if origin.strip()]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include authentication router
# Authentication router shamil karein
app.include_router(auth_router)

# Include chat history router
# Chat history router shamil karein
app.include_router(chat_router)

# --- Pydantic Models for Request Body ---
# These models define the expected structure of the JSON we receive in POST requests.
# Yeh models POST requests mein milne wale JSON ka expected structure define karte hain.

class QueryRequest(BaseModel):
    question: str
    selected_text: str = None  # Optional field (Yeh zaroori nahi hai)

class IngestRequest(BaseModel):
    force_reingest: bool = False # If true, clears existing data and re-ingests all content.

# --- API Endpoints ---

@app.get("/health", tags=["Status"])
def health_check():
    """
    Health Check Endpoint:
    Ensures the server is up and running.
    Server up and running hai, yeh یقینی banata hai.
    """
    return {"status": "ok", "message": "Server is healthy"}

@app.post("/query", tags=["RAG"])
async def query_endpoint(
    request: QueryRequest,
    authorization: str = None,
    db: Session = Depends(get_db)
):
    """
    Query Endpoint:
    Receives a question and optional selected text, and returns an answer and source chunks from the RAG pipeline.
    Optionally saves the conversation to database if user is authenticated.
    Ek sawal aur optional selected text receive karta hai, aur RAG pipeline se jawab aur source chunks deta hai.
    Agar user authenticated hai to conversation ko database mein save bhi kar deta hai.
    """
    from fastapi import Header
    from typing import Optional
    from db.models import ChatMessage, User
    from auth.utils import get_user_from_token
    from datetime import datetime

    try:
        # Call the main RAG pipeline
        # Main RAG pipeline ko call karo
        result = query_rag_pipeline(request.question, request.selected_text)

        # Save message asynchronously (don't block response)
        # Message ko asynchronously save karo (response ko block mat karo)
        import asyncio

        async def save_message_async():
            try:
                if authorization and authorization.startswith("Bearer "):
                    token = authorization.replace("Bearer ", "")
                    user_data = get_user_from_token(token)

                    if user_data:
                        # Save chat message to database
                        new_message = ChatMessage(
                            user_id=user_data["id"],
                            question=request.question,
                            answer=result.get("answer", ""),
                            source_chunks=result.get("source_chunks", []),
                            selected_text=request.selected_text,
                            created_at=datetime.utcnow()
                        )
                        db.add(new_message)
                        db.commit()
                        print(f"Chat message saved for user {user_data['id']}")
            except Exception as save_error:
                print(f"Failed to save chat message: {save_error}")
                db.rollback()

        # Fire and forget - save in background
        asyncio.create_task(save_message_async())

        return result # result is now {"answer": ..., "source_chunks": ...}
    except ValueError as e: # Catch ValueError specifically for API key issues
        print(f"Configuration Error in /query: {e}")
        raise HTTPException(
            status_code=401, # Unauthorized
            detail="Add your free key. Please ensure GOOGLE_API_KEY is set in your environment."
        )
    except Exception as e:
        # If anything goes wrong, return an error.
        # Agar kuch bhi galat ho, to error return karo.
        print(f"An error occurred in /query: {e}")
        # The 500 status code means "Internal Server Error"
        # 500 status code ka matlab hai "Internal Server Error"
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/ingest", tags=["Admin"])
async def ingest_endpoint(request: IngestRequest): # Changed to async as ingestion can be long-running
    """
    Ingestion Endpoint:
    Triggers the data ingestion process from the /docs directory.
    `/docs` directory se data ingestion process ko trigger karta hai.
    """
    try:
        # Construct the correct path to the documentation directory
        # Documentation directory ka sahi path banayein
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        # Corrected path to point to the Docusaurus docs folder
        docs_dir = os.path.join(project_root, 'Robotic book', 'docs')

        # Check if the directory exists
        if not os.path.isdir(docs_dir):
            raise HTTPException(status_code=404, detail=f"Documentation directory not found at: {docs_dir}")
        
        ingest_data(docs_path=docs_dir, collection_name="physical_ai_book")
        
        return {"status": "success", "message": f"Data from '{docs_dir}' ingested successfully into Qdrant and Neon Postgres."}
    except Exception as e:
        print(f"An error occurred during ingestion: {e}")
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")

# To run this app:
# 1. Open your terminal (Terminal kholo)
# 2. Navigate to the `backend` directory (`backend` directory mein jao)
# 3. Run the command: uvicorn app:app --reload
#    (Command chalao: uvicorn app:app --reload)
