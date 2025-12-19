# Murad... Asking for brother's help...
# This file takes the user's question and the documents we found, and asks Gemini to generate an answer.

import os
import time
from typing import List, Dict, Any
from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.prompts import PromptTemplate
from langchain_core.output_parsers import StrOutputParser
from dotenv import load_dotenv
from google.api_core.exceptions import ResourceExhausted
import google.generativeai as genai

load_dotenv()

_llm_instance = None

def list_available_models():
    print("\n--- Listing Available Gemini Models ---")
    try:
        google_api_key = os.getenv("GOOGLE_API_KEY")
        if not google_api_key:
            print("GOOGLE_API_KEY not found in .env file.")
            return

        genai.configure(api_key=google_api_key)
        
        for m in genai.list_models():
            if 'generateContent' in m.supported_generation_methods:
                print(f"Model found: {m.name}")
        print("--- Finished Listing Models ---\n")

    except Exception as e:
        print(f"Could not list models. Error: {e}")
        print("--- Finished Listing Models ---\n")


def get_llm():
    global _llm_instance
    if _llm_instance is None:
        list_available_models()

        google_api_key = os.getenv("GOOGLE_API_KEY")
        if not google_api_key:
            raise ValueError("GOOGLE_API_KEY must be set in the .env file to use the generator.")
        
        print("Initializing Gemini LLM (gemini-2.0-flash) with optimized settings...")
        _llm_instance = ChatGoogleGenerativeAI(
            model="gemini-2.0-flash",
            google_api_key=google_api_key,
            temperature=0.3,  # Lower temperature for faster, more focused responses
            max_output_tokens=512,  # Limit token count for faster responses
        )
        print("Gemini LLM initialized successfully.")
    return _llm_instance

# --- Prompt Template ---
PROMPT_TEMPLATE = """
You are a professional AI assistant specializing in Physical AI and Humanoid Robotics.

CRITICAL INSTRUCTIONS FOR GREETINGS:
- If user says ONLY: "hello", "hi", "hey", "good morning", "good afternoon", or "good evening"
- You MUST respond EXACTLY with this professional greeting:

"Hello! I'm your AI assistant for Physical AI & Humanoid Robotics. I can help you understand concepts about ROS 2, Digital Twins, Reinforcement Learning, and Vision-Language-Action models. How can I help you today?"

DO NOT just say "hi" or "hello" back. ALWAYS give the full professional greeting above.

For technical questions:
- Use the context provided to give accurate, detailed answers
- Be professional, clear, and educational
- Use examples from the context when available

If context is insufficient:
- Suggest related topics: ROS 2, NVIDIA Isaac Sim, RL algorithms, or VLA architectures

Context from the book:
{context}

User's Question/Input:
{question}

Your Answer:
"""

def generate_answer(question: str, context_documents: List[Dict[str, Any]]) -> str:
    llm_instance = get_llm()

    context_str = "\n---\n".join([doc.get('text_content', '') for doc in context_documents])
    
    prompt = PromptTemplate.from_template(PROMPT_TEMPLATE)
    
    rag_chain = prompt | llm_instance | StrOutputParser()
    
    for attempt in range(3):
        try:
            answer = rag_chain.invoke({
                "question": question,
                "context": context_str
            })
            return answer
        except ResourceExhausted as e:
            print(f"Rate limit hit, attempt {attempt + 1}/3. Retrying in 2 seconds...")
            time.sleep(2)
        except Exception as e:
            print(f"An unexpected error occurred during generation: {e}")
            raise

    return "Sorry, the service is currently busy. Please try again in a moment."


if __name__ == '__main__':
    dummy_docs = [
        {'id': '1', 'text_content': 'ROS 2 is a flexible framework for writing robot software.', 'metadata': {'source_file': 'ros2.mdx'}},
        {'id': '2', 'text_content': 'A digital twin is a virtual model of a physical object.', 'metadata': {'source_file': 'digital_twin.mdx'}},
    ]
    
    test_question = "What is ROS 2?"
    
    try:
        generated_answer = generate_answer(test_question, dummy_docs)
        print("\n--- Question ---")
        print(test_question)
        print("\n--- Generated Answer ---")
        print(generated_answer)

    except ValueError as e:
        print(f"Configuration Error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
