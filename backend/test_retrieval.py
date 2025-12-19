# Quick test script to verify Qdrant retrieval is working
# Yeh script check karne ke liye hai ke Qdrant retrieval theek se kaam kar raha hai

from rag.retriever import retrieve_relevant_chunks
from dotenv import load_dotenv

load_dotenv()

print("=" * 60)
print("Testing Qdrant Retrieval")
print("=" * 60)

# Test different questions
test_questions = [
    "how many modules in this book",
    "what is ros 2",
    "explain digital twin",
    "what are the main topics covered",
]

for question in test_questions:
    print(f"\n{'='*60}")
    print(f"Question: {question}")
    print(f"{'='*60}")

    try:
        results = retrieve_relevant_chunks(question)

        if results:
            print(f"✓ Found {len(results)} relevant chunks\n")
            for i, chunk in enumerate(results):
                print(f"Chunk {i+1}:")
                print(f"  Source: {chunk['metadata'].get('source_file', 'unknown')}")
                print(f"  Preview: {chunk['text_content'][:150]}...")
                print()
        else:
            print("✗ No results found!")
            print("   Possible issues:")
            print("   - Data not ingested in Qdrant")
            print("   - score_threshold too high")
            print("   - Question doesn't match book content")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

print("\n" + "=" * 60)
print("Test Complete")
print("=" * 60)
