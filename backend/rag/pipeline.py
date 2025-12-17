# Murad... Asking for brother's help...
# This file connects everything. It's the main pipeline for our RAG system.
# Yeh file sab kuch jodti hai. Yeh hamare RAG system ki main pipeline hai.

# Here's the flow (Kaam karne ka tarika):
# 1. User asks a question (User ek sawal puchta hai).
# 2. We turn the question into numbers (embeddings) using `embeddings.py`. (Hum `embeddings.py` ka istemal karke sawal ko numbers (embeddings) mein badalte hain).
# 3. We use these numbers to find relevant text from our book in Qdrant using `retriever.py`. (Hum in numbers ka istemal karke `retriever.py` ke zariye Qdrant mein hamari kitab se relevant text dhoondte hain).
# 4. We give the question and the found text to Gemini to get a final answer using `generator.py`. (Hum sawal aur mila hua text Gemini ko dekar `generator.py` ke zariye final jawab hasil karte hain).

from typing import List, Dict, Any, Optional

from rag.retriever import retrieve_relevant_chunks # Updated import
from rag.generator import generate_answer
from utils.embeddings import get_gemini_embedding # Updated import

def query_rag_pipeline(question: str, selected_text: Optional[str] = None) -> Dict[str, Any]:
    """
    Executes the full RAG pipeline, handling selected text override.
    Poore RAG pipeline ko chalata hai, selected text override ko handle karte hue.
    
    Args:
        question (str): The user's question. (User ka sawal)
        selected_text (Optional[str]): Text selected by the user in the frontend. (Frontend mein user dwara chuna gaya text)
    
    Returns:
        Dict[str, Any]: A dictionary containing the final answer and source chunks.
                        (Ek dictionary jismein final jawab aur source chunks shamil hain)
    """
    print("--- Starting RAG Pipeline ---")
    
    context_for_generation: List[Dict[str, Any]] = []
    response_source_chunks: List[Dict[str, Any]] = []
    final_answer: str = ""

    # 1. Handle selected text (Agar user ne text select kiya hai)
    if selected_text:
        print("User provided selected text. Skipping retrieval and using it as context.")
        # If user selects text, we use that as the ONLY context.
        # Agar user text select karta hai, to hum sirf usi ko context ke taur par istemal karte hain.
        context_for_generation = [{
            "id": "selected_text_override",
            "text_content": selected_text,
            "metadata": {"source_file": "user_selection", "chunk_index": 0}
        }]
        response_source_chunks = context_for_generation # For API response
    else:
        print("No selected text. Proceeding with retrieval from Qdrant.")
        # 2. Retrieve relevant documents from Qdrant
        # Qdrant se relevant documents retrieve karo
        # No need to generate embedding here; retriever handles it
        print(f"Retrieving documents from Qdrant for question: '{question}'")
        retrieved_chunks = retrieve_relevant_chunks(question)
        
        if not retrieved_chunks:
            print("No relevant chunks found in Qdrant for this query.")
            final_answer = "Sorry, I couldn't find any relevant information in the book to answer this question."
            return {"answer": final_answer, "source_chunks": []}
        
        print(f"Found {len(retrieved_chunks)} relevant documents.")
        context_for_generation = retrieved_chunks
        response_source_chunks = retrieved_chunks # For API response


    # 3. Generate an answer using the retrieved context or selected text
    # Retrieve kiye gaye context ya selected text ka istemal karke jawab generate karo
    print("Generating final answer with Gemini...")
    final_answer = generate_answer(question, context_for_generation)
    print("--- RAG Pipeline Finished ---")
    
    return {"answer": final_answer, "source_chunks": response_source_chunks}

# Example Usage
if __name__ == '__main__':
    # Ensure GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY are set in your .env
    # Yeh یقینی banayein ke GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY aapke .env mein set hain

    test_question_1 = "What is a digital twin?"
    print(f"Testing pipeline with question: '{test_question_1}' (Normal retrieval)")
    
    try:
        # Test case 1: Normal retrieval
        result_1 = query_rag_pipeline(test_question_1)
        print("\n--- Final Answer (from retrieval) ---")
        print(result_1["answer"])
        print("\n--- Source Chunks (from retrieval) ---")
        for chunk in result_1["source_chunks"]:
            print(f"- {chunk['metadata'].get('source_file')}: {chunk['text_content'][:50]}...")
        
        # Test case 2: With selected text
        print("\n" + "="*50 + "\n")
        selected_context = "A digital twin is simply a virtual, dynamic model of a physical object or system. It is updated with real-world data."
        test_question_2 = "Explain digital twin based on this text."
        print(f"Testing pipeline with selected text: '{selected_context}'")
        result_2 = query_rag_pipeline(test_question_2, selected_text=selected_context)
        print("\n--- Final Answer (from selected text) ---")
        print(result_2["answer"])
        print("\n--- Source Chunks (from selected text) ---")
        for chunk in result_2["source_chunks"]:
            print(f"- {chunk['metadata'].get('source_file')}: {chunk['text_content'][:50]}...")

    except ValueError as e:
        print(f"\nConfiguration Error: {e}")
        print("Please check your .env file and ensure GOOGLE_API_KEY, QDRANT_URL, QDRANT_API_KEY are correct.")
    except Exception as e:
        print(f"\nAn unexpected error occurred during the pipeline test: {e}")
        import traceback
        traceback.print_exc()
        print("Please ensure your Qdrant instance is running and accessible.")
