"""
Simplified Ingestion - Only Qdrant (No Database)
This is faster and avoids timeout issues
"""
import os
import sys
import glob
from uuid import uuid4

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import TextLoader
from qdrant_client import models
from utils.embeddings import get_gemini_embedding
from utils.qdrant_connection import get_qdrant_client

def simple_ingest():
    print("=" * 60)
    print("SIMPLE DATA INGESTION - QDRANT ONLY")
    print("=" * 60)

    # Paths
    backend_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(backend_dir)
    docs_path = os.path.join(project_root, 'Robotic book', 'docs')

    print(f"\n1. Looking for docs at: {docs_path}")

    if not os.path.exists(docs_path):
        print(f"ERROR: Docs directory not found!")
        return

    # Find all .mdx files
    mdx_files = glob.glob(os.path.join(docs_path, '**/*.mdx'), recursive=True)
    print(f"   Found {len(mdx_files)} .mdx files")

    if not mdx_files:
        print("   No files found! Check the path.")
        return

    # Load documents
    print("\n2. Loading documents...")
    all_docs = []
    for file_path in mdx_files:
        try:
            loader = TextLoader(file_path, encoding='utf-8')
            docs = loader.load()
            all_docs.extend(docs)
            print(f"   ✓ Loaded: {os.path.basename(file_path)}")
        except Exception as e:
            print(f"   ✗ Failed: {os.path.basename(file_path)} - {e}")

    if not all_docs:
        print("   No documents loaded!")
        return

    # Split into chunks
    print(f"\n3. Splitting {len(all_docs)} documents into chunks...")
    splitter = RecursiveCharacterTextSplitter(
        chunk_size=800,
        chunk_overlap=100,
        separators=["\n\n", "\n", ".", " ", ""]
    )
    all_chunks = splitter.split_documents(all_docs)
    print(f"   Created {len(all_chunks)} chunks")

    # Get embedding function
    print("\n4. Initializing embeddings...")
    embedding_func = get_gemini_embedding()

    # Create embeddings
    print("\n5. Creating embeddings (this takes time)...")
    points = []

    for i, chunk in enumerate(all_chunks):
        if i % 10 == 0:
            print(f"   Processing chunk {i+1}/{len(all_chunks)}...")

        try:
            chunk_id = str(uuid4())
            embedding = embedding_func.embed_query(chunk.page_content)

            metadata = {
                "source_file": os.path.basename(chunk.metadata.get('source', 'unknown')),
                **chunk.metadata
            }

            points.append(models.PointStruct(
                id=chunk_id,
                vector=embedding,
                payload={
                    "text_content": chunk.page_content,
                    "metadata": metadata
                }
            ))
        except Exception as e:
            print(f"   ✗ Error on chunk {i}: {e}")

    print(f"   ✓ Created {len(points)} embeddings")

    # Upload to Qdrant
    print("\n6. Uploading to Qdrant...")
    qdrant_client = get_qdrant_client()
    collection_name = "physical_ai_book"

    # Recreate collection
    print(f"   Creating collection: '{collection_name}'")
    vector_size = len(points[0].vector) if points else 768
    qdrant_client.recreate_collection(
        collection_name=collection_name,
        vectors_config=models.VectorParams(
            size=vector_size,
            distance=models.Distance.COSINE
        ),
    )

    # Upload in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i+batch_size]
        qdrant_client.upsert(
            collection_name=collection_name,
            wait=True,
            points=batch
        )
        print(f"   Uploaded batch {i//batch_size + 1}/{(len(points) + batch_size - 1)//batch_size}")

    print("\n" + "=" * 60)
    print("SUCCESS! Data ingested into Qdrant")
    print(f"Total points: {len(points)}")
    print("=" * 60)
    print("\nYou can now start the backend and use the chatbot!")
    print("Command: uvicorn app:app --reload")

if __name__ == "__main__":
    try:
        simple_ingest()
    except Exception as e:
        print("\n" + "=" * 60)
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        print("=" * 60)
