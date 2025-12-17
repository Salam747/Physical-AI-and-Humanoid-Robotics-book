
import os
from qdrant_client import QdrantClient
from dotenv import load_dotenv

load_dotenv()

def inspect_qdrant_client():
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not qdrant_url:
        print("QDRANT_URL environment variable not set.")
        return

    try:
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
        )
        print(f"QdrantClient object type: {type(client)}")
        print("\nAvailable attributes and methods for QdrantClient:")
        for attr in sorted(dir(client)):
            if not attr.startswith('_'): # Exclude private/protected members
                print(f"- {attr}")
        
        # Also try to check if 'search' is directly callable
        if hasattr(client, 'search'):
            print("\n'search' method IS present on QdrantClient.")
        else:
            print("\n'search' method is NOT present on QdrantClient.")

    except Exception as e:
        print(f"An error occurred during QdrantClient inspection: {e}")

if __name__ == "__main__":
    inspect_qdrant_client()
