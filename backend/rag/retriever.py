# Murad... Asking for brother's help...
# This file is for searching and finding similar documents from our vector database (Qdrant).
# Yeh file hamare vector database (Qdrant) se milte julte documents dhoondne ke liye hai.

import os
from typing import List, Dict, Any
from qdrant_client import models, QdrantClient

from utils.embeddings import get_gemini_embedding
# from utils.qdrant_connection import get_qdrant_client # Bypassing for diagnostics

def retrieve_relevant_chunks(user_query: str, collection_name: str = "physical_ai_book", top_k: int = 5, score_threshold: float = 0.4) -> List[Dict[str, Any]]:
    """
    Generates an embedding for the user query and searches for the most similar documents
    in a Qdrant collection, filtering by score and limiting to top_k.

    User query ke liye embedding banata hai aur Qdrant collection mein sabse similar documents dhoondta hai,
    score se filter karta hai aur top_k tak seemit karta hai.

    Args:
        user_query (str): The user's natural language query. (User ka natural language query)
        collection_name (str): The name of the collection to search in. (Collection ka naam jahan search karna hai)
        top_k (int): The number of top results to return. (Kitne top results wapas karne hain)
        score_threshold (float): Minimum similarity score for a chunk to be considered relevant.
                                 (Chunk ko relevant manne ke liye minimum similarity score)

    Returns:
        List[Dict[str, Any]]: A list of dictionaries, each representing a relevant chunk
                               with 'id', 'text_content', and 'metadata'.
                               (Dictionaries ki list, har ek 'id', 'text_content', aur 'metadata' ke sath relevant chunk ko darshati hai)
    """
    # 1. Get embedding function
    # Embedding function hasil karo
    embedding_func = get_gemini_embedding()

    # --- DIAGNOSTIC: Create a new Qdrant client instance directly ---
    print("--- DIAGNOSTIC: Creating new QdrantClient directly in retriever ---")
    qdrant_client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY"),
    )
    # ---------------------------------------------------------------

    # 2. Generate embedding for the user query
    # User query ke liye embedding banayein
    query_embedding = embedding_func.embed_query(user_query)

    # 3. Search Qdrant for relevant chunks using query_points (new API)
    # Relevant chunks ke liye Qdrant mein search karein
    search_results = qdrant_client.query_points(
        collection_name=collection_name,
        query=query_embedding,
        limit=top_k,
        score_threshold=score_threshold,
        with_payload=True
    )

    # 4. Format and return results
    # Results ko format karein aur wapas karein
    relevant_chunks = []
    for hit in search_results.points:
        relevant_chunks.append({
            "id": hit.id,
            "text_content": hit.payload.get('text_content'),
            "metadata": hit.payload.get('metadata')
        })

    return relevant_chunks

# Example usage to test the retriever
if __name__ == '__main__':
    # Ensure GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY are set in your .env
    # Yeh یقینی banayein ke GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY aapke .env mein set hain

    sample_query = "What is Physical AI and how does it relate to robotics?"
    print(f"Searching for chunks related to: '{sample_query}'")

    try:
        results = retrieve_relevant_chunks(sample_query)

        if results:
            print(f"Found {len(results)} relevant chunks:")
            for i, chunk in enumerate(results):
                print(f"\n--- Chunk {i+1} (ID: {chunk['id']}) ---")
                print(f"Source: {chunk['metadata'].get('source_file')}")
                print(f"Content Snippet: {chunk['text_content'][:200]}...")
        else:
            print("No relevant chunks found. Ensure the Qdrant collection 'physical_ai_book' exists and has data.")

    except ValueError as e:
        print(f"Configuration Error: {e}")
        print("Please check your .env file and ensure GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY are correct.")
    except Exception as e:
        print(f"An error occurred: {e}")
        import traceback
        traceback.print_exc()
        print("Please ensure your Qdrant instance is running and accessible.")
