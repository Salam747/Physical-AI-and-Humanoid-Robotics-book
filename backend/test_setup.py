#!/usr/bin/env python
"""
Backend Configuration Test Script
Tests all components needed for RAG chatbot to work properly
"""

import os
import sys
from dotenv import load_dotenv

# Color codes for terminal output
GREEN = '\033[92m'
RED = '\033[91m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
RESET = '\033[0m'

def print_status(message, status):
    """Print colored status message"""
    if status == "OK":
        print(f"{GREEN}✓{RESET} {message}: {GREEN}{status}{RESET}")
    elif status == "WARN":
        print(f"{YELLOW}⚠{RESET} {message}: {YELLOW}WARNING{RESET}")
    else:
        print(f"{RED}✗{RESET} {message}: {RED}{status}{RESET}")

def test_environment_variables():
    """Test if all required environment variables are set"""
    print(f"\n{BLUE}=== Testing Environment Variables ==={RESET}")

    load_dotenv()

    required_vars = {
        'GOOGLE_API_KEY': 'Gemini API for AI responses',
        'DATABASE_URL': 'PostgreSQL for user data',
        'QDRANT_URL': 'Vector DB for book content',
        'QDRANT_API_KEY': 'Qdrant authentication',
        'JWT_SECRET_KEY': 'User authentication',
    }

    all_ok = True
    for var, description in required_vars.items():
        value = os.getenv(var)
        if value and len(value) > 5:
            print_status(f"{var} ({description})", "OK")
        else:
            print_status(f"{var} ({description})", "MISSING")
            all_ok = False

    return all_ok

def test_imports():
    """Test if all required modules can be imported"""
    print(f"\n{BLUE}=== Testing Python Module Imports ==={RESET}")

    modules = [
        ('fastapi', 'FastAPI web framework'),
        ('uvicorn', 'ASGI server'),
        ('qdrant_client', 'Qdrant vector DB client'),
        ('google.generativeai', 'Google Gemini AI'),
        ('psycopg2', 'PostgreSQL driver'),
        ('jose', 'JWT token handling'),
        ('passlib', 'Password hashing'),
    ]

    all_ok = True
    for module, description in modules:
        try:
            __import__(module)
            print_status(f"{module} ({description})", "OK")
        except ImportError:
            print_status(f"{module} ({description})", "NOT INSTALLED")
            all_ok = False

    return all_ok

def test_rag_pipeline():
    """Test if RAG pipeline components are working"""
    print(f"\n{BLUE}=== Testing RAG Pipeline Components ==={RESET}")

    try:
        from rag.pipeline import query_rag_pipeline
        print_status("RAG Pipeline Import", "OK")
    except Exception as e:
        print_status("RAG Pipeline Import", f"FAILED: {e}")
        return False

    try:
        from utils.embeddings import get_gemini_embedding
        print_status("Embeddings Module", "OK")
    except Exception as e:
        print_status("Embeddings Module", f"FAILED: {e}")
        return False

    try:
        from rag.retriever import retrieve_relevant_chunks
        print_status("Retriever Module", "OK")
    except Exception as e:
        print_status("Retriever Module", f"FAILED: {e}")
        return False

    try:
        from rag.generator import generate_answer
        print_status("Generator Module", "OK")
    except Exception as e:
        print_status("Generator Module", f"FAILED: {e}")
        return False

    return True

def test_database_connection():
    """Test database connectivity"""
    print(f"\n{BLUE}=== Testing Database Connection ==={RESET}")

    try:
        import psycopg2
        from dotenv import load_dotenv
        load_dotenv()

        db_url = os.getenv('DATABASE_URL')
        if not db_url:
            print_status("Database Connection", "NO DATABASE_URL")
            return False

        conn = psycopg2.connect(db_url)
        conn.close()
        print_status("PostgreSQL Connection", "OK")
        return True
    except Exception as e:
        print_status("PostgreSQL Connection", f"FAILED: {str(e)[:50]}")
        return False

def test_qdrant_connection():
    """Test Qdrant connection"""
    print(f"\n{BLUE}=== Testing Qdrant Vector DB ==={RESET}")

    try:
        from utils.qdrant_connection import get_qdrant_client

        client = get_qdrant_client()
        collections = client.get_collections()
        print_status("Qdrant Connection", "OK")
        print(f"  Collections: {[col.name for col in collections.collections]}")
        return True
    except Exception as e:
        print_status("Qdrant Connection", f"FAILED: {str(e)[:50]}")
        return False

def test_gemini_api():
    """Test Gemini API"""
    print(f"\n{BLUE}=== Testing Gemini AI API ==={RESET}")

    try:
        import google.generativeai as genai
        from dotenv import load_dotenv
        load_dotenv()

        api_key = os.getenv('GOOGLE_API_KEY')
        if not api_key:
            print_status("Gemini API", "NO API KEY")
            return False

        genai.configure(api_key=api_key)
        # Try different model names
        model_names = ['gemini-pro', 'gemini-1.5-pro', 'gemini-1.0-pro']
        response = None
        for model_name in model_names:
            try:
                model = genai.GenerativeModel(model_name)
                response = model.generate_content("Say 'OK' if you can read this.")
                if response and response.text:
                    break
            except:
                continue

        if response and response.text:
            print_status("Gemini API Connection", "OK")
            print(f"  Test Response: {response.text[:50]}")
            return True
        else:
            print_status("Gemini API", "NO RESPONSE")
            return False
    except Exception as e:
        print_status("Gemini API Connection", f"FAILED: {str(e)[:50]}")
        return False

def main():
    """Run all tests"""
    print(f"\n{BLUE}{'='*60}{RESET}")
    print(f"{BLUE}  RAG Chatbot Backend Configuration Test{RESET}")
    print(f"{BLUE}{'='*60}{RESET}")

    results = {
        'Environment Variables': test_environment_variables(),
        'Module Imports': test_imports(),
        'RAG Pipeline': test_rag_pipeline(),
        'Database Connection': test_database_connection(),
        'Qdrant Vector DB': test_qdrant_connection(),
        'Gemini AI API': test_gemini_api(),
    }

    print(f"\n{BLUE}=== Test Summary ==={RESET}")
    all_passed = True
    for test_name, passed in results.items():
        if passed:
            print_status(test_name, "PASSED")
        else:
            print_status(test_name, "FAILED")
            all_passed = False

    print(f"\n{BLUE}{'='*60}{RESET}")
    if all_passed:
        print(f"{GREEN}✓ All tests passed! Your backend is ready.{RESET}")
        print(f"\n{BLUE}Next steps:{RESET}")
        print("1. Start the server: uvicorn app:app --reload")
        print("2. Test the API: http://127.0.0.1:8000/health")
        print("3. Ingest data: POST http://127.0.0.1:8000/ingest")
        print("4. Start frontend: cd 'Robotic book' && npm start")
    else:
        print(f"{RED}✗ Some tests failed. Please fix the issues above.{RESET}")
        return 1

    print(f"{BLUE}{'='*60}{RESET}\n")
    return 0

if __name__ == "__main__":
    sys.exit(main())
