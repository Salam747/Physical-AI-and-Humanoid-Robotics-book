# Murad... Asking for brother's help...
# This is the main server file using FastAPI. It creates the API endpoints.
# Yeh FastAPI ka istemal karke main server file hai. Yeh API endpoints banati hai.

# Endpoints (API ke raaste):
# - GET /health: Checks if the server is running. (Server chal raha hai ya nahi, yeh check karta hai).
# - POST /query: The main endpoint to ask questions. (Sawal puchne ke liye main endpoint).
# - POST /ingest: To load the book content into the database (run only once). (Kitab ka content database mein load karne ke liye - sirf ek baar chalana hai).

import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware # Import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv

# Import the main RAG pipeline function
# Main RAG pipeline function import karo
from rag.pipeline import query_rag_pipeline

# Import the ingestion script function (we will create this file next)
# Ingestion script function import karo (hum yeh file iske baad banayenge)
from ingest.ingest_data import ingest_data

# Import authentication router
# Authentication router import karo
from auth.routes import router as auth_router

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
origins = [
    "https://salam747.github.io",  # GitHub Pages domain
    "http://localhost",
    "http://localhost:3000",  # Default for many frontend development servers
    "http://localhost:3001",  # Docusaurus default port
    "http://127.0.0.1:3000",
    "http://127.0.0.1:3001",
]

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
async def query_endpoint(request: QueryRequest):
    """
    Query Endpoint:
    Receives a question and optional selected text, and returns an answer and source chunks from the RAG pipeline.
    Ek sawal aur optional selected text receive karta hai, aur RAG pipeline se jawab aur source chunks deta hai.
    """
    try:
        # Call the main RAG pipeline
        # Main RAG pipeline ko call karo
        result = query_rag_pipeline(request.question, request.selected_text)
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
