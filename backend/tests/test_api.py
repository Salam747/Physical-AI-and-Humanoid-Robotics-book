import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock
from fastapi import HTTPException

# Import the FastAPI app from the main app file
# Main app file se FastAPI app import karo
from app import app

# Create a test client to make requests to our app
# Hamare app ko requests bhejne ke liye ek test client banayein
client = TestClient(app)

def test_health_check():
    """
    Tests the /health endpoint.
    /health endpoint ko test karta hai.
    """
    response = client.get("/health")
    # Assert that the request was successful (status code 200)
    # Tasdeeq karein ke request kamyab thi (status code 200)
    assert response.status_code == 200
    # Assert that the response body is what we expect
    # Tasdeeq karein ke response body wahi hai jo hum expect karte hain
    assert response.json() == {"status": "ok", "message": "Server is healthy"}

@patch('backend.app.query_rag_pipeline')
def test_query_endpoint_success(mock_query_rag_pipeline):
    """
    Tests the /query endpoint for a successful response with the new return format.
    /query endpoint ko naye return format ke sath kamyab response ke liye test karta hai.
    """
    # Configure the mock to return a specific value when called
    # Mock ko configure karein ke jab call kiya jaye to ek makhsoos value return kare
    expected_result = {
        "answer": "This is a successful mock answer.",
        "source_chunks": [{"id": "123", "text_content": "mock content", "metadata": {}}]
    }
    mock_query_rag_pipeline.return_value = expected_result
    
    # Make a POST request to the /query endpoint
    # /query endpoint par POST request bhejein
    response = client.post("/query", json={"question": "What is a test?"})
    
    # Assert that the request was successful
    # Tasdeeq karein ke request kamyab thi
    assert response.status_code == 200
    # Assert that the response body contains the expected result from our mock
    # Tasdeeq karein ke response body mein hamare mock se mutawaqqa result hai
    assert response.json() == expected_result
    # Assert that our mock function was called correctly
    # Tasdeeq karein ke hamara mock function sahi tarah se call hua tha
    mock_query_rag_pipeline.assert_called_once_with("What is a test?", None)

@patch('backend.app.query_rag_pipeline')
def test_query_endpoint_error(mock_query_rag_pipeline):
    """
    Tests how the /query endpoint handles a generic error from the pipeline (500 status).
    /query endpoint pipeline se aane wale aam error (500 status) ko kaise handle karta hai, yeh test karta hai.
    """
    # Configure the mock to raise an exception when called
    # Mock ko configure karein ke jab call kiya jaye to ek exception raise kare
    error_message = "Something went wrong in the pipeline"
    mock_query_rag_pipeline.side_effect = Exception(error_message) # Mock a generic Exception
    
    # Make a POST request
    # POST request bhejein
    response = client.post("/query", json={"question": "This will fail"})
    
    # Assert that the server responded with a 500 Internal Server Error
    # Tasdeeq karein ke server ne 500 Internal Server Error ke sath response diya
    assert response.status_code == 500
    # Assert that the error detail in the response matches our exception message
    # Tasdeeq karein ke response mein error detail hamare exception message se match karta hai
    assert response.json() == {"detail": error_message}

@patch('backend.app.query_rag_pipeline')
def test_query_with_selected_text(mock_query_rag_pipeline):
    """
    Tests that the endpoint correctly handles the 'selected_text' parameter with the new return format.
    Test karta hai ke endpoint 'selected_text' parameter ko naye return format ke sath sahi tarah se handle karta hai.
    """
    expected_result = {
        "answer": "Answer from selected text.",
        "source_chunks": [{"id": "456", "text_content": "selected content", "metadata": {}}]
    }
    mock_query_rag_pipeline.return_value = expected_result
    
    question = "What is this?"
    selected_text = "This is a specific context provided by the user."
    
    response = client.post("/query", json={"question": question, "selected_text": selected_text})
    
    assert response.status_code == 200
    assert response.json() == expected_result
    # Assert that the pipeline was called with both the question and the selected text
    # Tasdeeq karein ke pipeline question aur selected text dono ke sath call hui thi
    mock_query_rag_pipeline.assert_called_once_with(question, selected_text)

@patch('backend.rag.pipeline.get_gemini_embedding') # Patching where ValueError is raised
def test_query_api_key_error(mock_get_gemini_embedding):
    """
    Tests that the /query endpoint correctly handles missing API key errors (401 status).
    Test karta hai ke /query endpoint missing API key errors (401 status) ko sahi tarah se handle karta hai.
    """
    # Configure the mock to raise a ValueError (simulating missing GOOGLE_API_KEY)
    # Mock ko configure karein ke woh ValueError raise kare (missing GOOGLE_API_KEY ko simulate karte hue)
    mock_get_gemini_embedding.side_effect = ValueError("GOOGLE_API_KEY environment variable not set.")
    
    response = client.post("/query", json={"question": "Any question"})
    
    # Assert that the server responded with a 401 Unauthorized error
    # Tasdeeq karein ke server ne 401 Unauthorized error ke sath response diya
    assert response.status_code == 401
    # Assert that the error detail matches our custom message
    # Tasdeeq karein ke error detail hamare custom message se match karta hai
    assert response.json() == {"detail": "Add your free key. Please ensure GOOGLE_API_KEY is set in your environment."}

