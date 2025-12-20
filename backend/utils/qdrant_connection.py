import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Global Qdrant client instance to avoid re-initialization
_qdrant_client = None

def get_qdrant_client() -> QdrantClient:
    """
    Initializes and returns a Qdrant client instance.
    The client is configured using QDRANT_URL and QDRANT_API_KEY from environment variables.
    If the client is already initialized, the existing instance is returned.

    Qdrant client ki instance ko initialize aur return karta hai.
    Client environment variables se QDRANT_URL aur QDRANT_API_KEY ka istemal karke configure hota hai.
    Agar client pehle se initialize hai, to maujooda instance wapas ki jayegi.
    """
    global _qdrant_client
    if _qdrant_client is None:
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if not qdrant_url:
            raise ValueError("QDRANT_URL environment variable not set.")
            # QDRANT_URL environment variable set nahi hai.

        # Initialize Qdrant client with timeout settings
        _qdrant_client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            timeout=30,  # Increased timeout to 30 seconds
        )
    return _qdrant_client

if __name__ == "__main__":
    # Example usage (testing)
    # Istemal ka tareeqa (testing)
    try:
        client = get_qdrant_client()
        print("Qdrant client initialized successfully!")
        # You can add a test call here, e.g., client.get_collections()
        # Yahan aap ek test call shamil kar sakte hain, maslan client.get_collections()
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
