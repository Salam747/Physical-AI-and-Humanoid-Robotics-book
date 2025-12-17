# Murad... Asking for brother's help...
# This file handles how we turn words into numbers (embeddings).
# Yeh file dekhti hai ke hum lafzon ko numbers (embeddings) mein kaise badalte hain.

import os
from langchain_google_genai import GoogleGenerativeAIEmbeddings
# from langchain.embeddings import HuggingFaceEmbeddings # Removed as per strict Gemini API usage
from dotenv import load_dotenv

# Load environment variables from .env file
# .env file se environment variables load karo
load_dotenv()

def get_gemini_embedding() -> GoogleGenerativeAIEmbeddings:
    """
    Initializes and returns a Gemini embedding function using text-embedding-004 model.
    Requires GOOGLE_API_KEY environment variable to be set.

    text-embedding-004 model ka istemal karte hue Gemini embedding function ko initialize aur return karta hai.
    Iske liye GOOGLE_API_KEY environment variable ka set hona zaroori hai.
    """
    google_api_key = os.getenv("GOOGLE_API_KEY")

    if not google_api_key:
        raise ValueError("GOOGLE_API_KEY environment variable not set. Please set it to use Gemini embeddings.")
        # GOOGLE_API_KEY environment variable set nahi hai. Baraye meherbani Gemini embeddings istemal karne ke liye ise set karein.

    print("Using Gemini API for embeddings with text-embedding-004.")
    return GoogleGenerativeAIEmbeddings(model="models/text-embedding-004", google_api_key=google_api_key)

# Example usage:
if __name__ == '__main__':
    # Ensure GOOGLE_API_KEY is set in your .env or environment for this example to work.
    # Is misal ke kaam karne ke liye یقینی banayein ke aapka GOOGLE_API_KEY aapke .env ya environment mein set hai.
    try:
        embedding_function = get_gemini_embedding()

        # Create embeddings for a sample text
        # Sample text ke liye embeddings banayein
        sample_text = "Hello, this is a test for the Gemini embedding function."
        embeddings = embedding_function.embed_query(sample_text)

        print(f"Successfully generated embeddings. Vector dimension: {len(embeddings)}")
        # print(embeddings)
    except ValueError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

