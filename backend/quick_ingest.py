"""
Quick Ingestion Script
Run this directly to ingest book data into Qdrant
"""
import os
import sys

# Add backend directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ingest.ingest_data import ingest_data

if __name__ == "__main__":
    print("Starting data ingestion...")
    print("This may take 2-5 minutes depending on book size.")
    print("-" * 60)

    # Get the correct path to docs
    backend_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(backend_dir)
    docs_path = os.path.join(project_root, 'Robotic book', 'docs')

    print(f"Looking for docs at: {docs_path}")

    if not os.path.exists(docs_path):
        print(f"ERROR: Docs directory not found at {docs_path}")
        sys.exit(1)

    try:
        ingest_data(docs_path=docs_path, collection_name="physical_ai_book")
        print("-" * 60)
        print("SUCCESS! Data ingested successfully.")
        print("You can now use the chatbot to ask questions about the book!")
    except Exception as e:
        print("-" * 60)
        print(f"ERROR during ingestion: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
