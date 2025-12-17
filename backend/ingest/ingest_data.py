# Murad... Asking for brother's help...
# This file is for loading the book content into our vector database (Qdrant).
# Yeh file kitab ke content ko hamare vector database (Qdrant) mein load karne ke liye hai.

# How it works (Kaise kaam karta hai):
# 1. It reads a large text file (like our book). (Yeh ek badi text file (jaise hamari kitab) ko padhta hai).
# 2. It splits the text into smaller, more manageable chunks. (Yeh text ko chote, manageable chunks mein todta hai).
# 3. It turns each chunk into numbers (embeddings) using our embedding function. (Yeh har chunk ko hamare embedding function ka istemal karke numbers (embeddings) mein badalta hai).
# 4. It uploads these chunks and their embeddings to the Qdrant cloud so we can search them later. (Yeh in chunks aur unke embeddings ko Qdrant cloud par upload karta hai taake hum unhe baad mein search kar sakein).

import os
import glob # For finding files
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import TextLoader
from qdrant_client import QdrantClient, models
from typing import List

from utils.embeddings import get_gemini_embedding # Updated import
from utils.qdrant_connection import get_qdrant_client
from db.models import BookChunk, get_db # Import BookChunk model and get_db session
from sqlalchemy.orm import Session # For type hinting

# Global embedding function instance
_embedding_func = None

def _get_or_create_embedding_func():
    global _embedding_func
    if _embedding_func is None:
        _embedding_func = get_gemini_embedding()
    return _embedding_func

def ingest_data(docs_path: str = "../../docs", collection_name: str = "physical_ai_book"):
    """
    Reads data from Markdown files in a directory, creates embeddings, and ingests into Qdrant and Neon Postgres.
    Ek directory mein maujood Markdown files se data padhta hai, embeddings banata hai, aur Qdrant aur Neon Postgres mein ingest karta hai.
    """
    print(f"--- Starting Ingestion from {docs_path} to collection '{collection_name}' ---")

    embedding_func = _get_or_create_embedding_func()

    # 1. Load all documents from the specified directory
    # Specified directory se sabhi documents load karo
    print(f"Searching for .mdx files in: {docs_path}")
    mdx_files = glob.glob(os.path.join(docs_path, '**/*.mdx'), recursive=True)
    if not mdx_files:
        print(f"No .mdx files found in '{docs_path}'. Exiting ingestion.")
        return

    all_docs = []
    for file_path in mdx_files:
        print(f"Loading document: {os.path.basename(file_path)}")
        loader = TextLoader(file_path, encoding='utf-8')
        all_docs.extend(loader.load())

    if not all_docs:
        print("No content loaded from documents. Exiting ingestion.")
        return

    # 2. Split the documents into chunks
    # Documents ko chunks mein todo
    print("Splitting documents into chunks...")
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=2000,
        chunk_overlap=200,
    )
    all_chunks = text_splitter.split_documents(all_docs)

    if not all_chunks:
        print("Could not split documents into any chunks. Exiting ingestion.")
        return
        
    print(f"Total of {len(all_chunks)} chunks generated from {len(mdx_files)} documents.")

    # 3. Prepare points for Qdrant and data for Postgres
    # Qdrant ke liye points aur Postgres ke liye data taiyar karein
    print("Generating embeddings and preparing data for databases...")
    points_for_qdrant = []
    chunks_for_postgres = []
    from uuid import uuid4

    # Safely get vector size from the first chunk's embedding
    # Pehle chunk ke embedding se vector size ko mehfooz tareeqe se hasil karein
    try:
        first_embedding = embedding_func.embed_query(all_chunks[0].page_content)
        vector_size = len(first_embedding)
        print(f"Detected vector size: {vector_size}")
    except Exception as e:
        print(f"Could not generate embedding to determine vector size. Error: {e}")
        return
        
    # Process first chunk since we already embedded it
    # Pehla chunk process karein kyunki humne use pehle hi embed kar liya hai
    chunk_id = str(uuid4())
    metadata = { "source_file": os.path.basename(all_chunks[0].metadata.get('source', 'unknown')), **all_chunks[0].metadata }
    points_for_qdrant.append(models.PointStruct(id=chunk_id, vector=first_embedding, payload={"text_content": all_chunks[0].page_content, "metadata": metadata}))
    chunks_for_postgres.append({"id": chunk_id, "text_content": all_chunks[0].page_content, "metadata": metadata})

    # Process the rest of the chunks
    # Baqi ke chunks process karein
    for chunk in all_chunks[1:]:
        chunk_id = str(uuid4())
        metadata = { "source_file": os.path.basename(chunk.metadata.get('source', 'unknown')), **chunk.metadata }
        embedding = embedding_func.embed_query(chunk.page_content)
        
        points_for_qdrant.append(models.PointStruct(id=chunk_id, vector=embedding, payload={"text_content": chunk.page_content, "metadata": metadata}))
        chunks_for_postgres.append({"id": chunk_id, "text_content": chunk.page_content, "metadata": metadata})

    # 4. Initialize Qdrant client and create collection
    # Qdrant client initialize karein aur collection banayein
    qdrant_client = get_qdrant_client()
    print(f"Creating or recreating Qdrant collection: '{collection_name}'")
    qdrant_client.recreate_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
    )
    print("Qdrant Collection created successfully.")

    # 5. Upload the points to Qdrant
    # Points ko Qdrant par upload karein
    print(f"Uploading {len(points_for_qdrant)} points to Qdrant...")
    qdrant_client.upsert(
        collection_name=collection_name,
        wait=True,
        points=points_for_qdrant
    )
    print("Uploaded points to Qdrant successfully.")

    # 6. Save chunk metadata to Neon Postgres
    # Chunk metadata ko Neon Postgres mein save karein
    print(f"Saving {len(chunks_for_postgres)} chunks to Neon Postgres...")
    db: Session
    for db in get_db(): # Use get_db() as a context manager
        try:
            db.query(BookChunk).delete() # Clear old data
            for chunk_data in chunks_for_postgres:
                book_chunk = BookChunk(
                    id=chunk_data["id"],
                    text_content=chunk_data["text_content"],
                    chunk_metadata=chunk_data["metadata"]
                )
                db.add(book_chunk)
            db.commit()
            print("Saved chunks to Neon Postgres successfully.")
        except Exception as e:
            db.rollback()
            print(f"Error saving to Postgres: {e}")
            raise
        finally:
            db.close()

    print(f"--- Ingestion Complete! {len(all_chunks)} chunks processed. ---")



# Example of how to run this script directly
# Is script ko seedhe chalane ka example
if __name__ == '__main__':
    # Assuming '/docs' is at the project root level
    # Yeh maan kar chal rahe hain ke '/docs' project root level par hai
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    docs_dir = os.path.join(project_root, 'docs')

    try:
        # Create a dummy docs directory and file for testing if they don't exist
        # Testing ke liye dummy docs directory aur file banayein agar woh maujood nahi hain
        dummy_docs_path = os.path.join(project_root, 'docs')
        os.makedirs(dummy_docs_path, exist_ok=True)
        dummy_file_path = os.path.join(dummy_docs_path, "test_book_content.mdx")
        
        if not os.path.exists(dummy_file_path):
            with open(dummy_file_path, "w", encoding="utf-8") as f:
                f.write("# Introduction to Physical AI\n\n")
                f.write("Physical AI combines robotics and artificial intelligence to create intelligent agents that interact with the physical world. This field is rapidly evolving, driving innovations in automation, healthcare, and exploration.\n\n")
                f.write("## The Role of Robotics\n\n")
                f.write("Robotics provides the physical embodiment and mechanisms for AI agents. From humanoid robots to industrial arms, the hardware is crucial for executing tasks in real environments.\n\n")
                f.write("## AI Brain\n\n")
                f.write("The AI brain, often powered by advanced machine learning models, enables robots to perceive, reason, learn, and make decisions. This includes computer vision, natural language processing, and reinforcement learning.\n\n")
                f.write("This content is about Module 1 and Module 2 of the book.\n")

        ingest_data(docs_path=docs_dir, collection_name="physical_ai_book")
        
        print("\nVerification: Check your Qdrant Cloud dashboard for 'physical_ai_book' collection and Neon Postgres for 'book_chunks' table.")
        
    except ValueError as e:
        print(f"\nConfiguration Error: {e}")
        print("Please check your .env file and ensure GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY, and NEON_DB_URL are correctly set.")
    except Exception as e:
        print(f"\nAn error occurred during the ingestion test: {e}")
        import traceback
        traceback.print_exc()
        print("Please ensure your database is accessible and Qdrant is running.")
    finally:
        # Clean up the dummy file, but keep the docs directory for potential future runs
        if os.path.exists(dummy_file_path):
            # os.remove(dummy_file_path) # Commented out to allow verification if needed
            print(f"Dummy file '{dummy_file_path}' exists. You can manually remove it after verification.")
