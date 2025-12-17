# Murad... Asking for brother's help...
# This file contains tests for the data ingestion process.
# Yeh file data ingestion process ke liye tests rakhti hai.

import pytest
from unittest.mock import patch, MagicMock
import os
import shutil # For cleaning up directories

# Import the function to be tested
# Test kiye jane wale function ko import karo
from ingest.ingest_data import ingest_data

@pytest.fixture
def temp_docs_dir(tmp_path):
    """
    A pytest fixture to create a temporary dummy docs directory with a .mdx file for testing.
    Testing ke liye ek temporary dummy docs directory aur .mdx file banane ke liye ek pytest fixture.
    """
    docs_dir = tmp_path / "docs"
    docs_dir.mkdir()
    file_path = docs_dir / "test_book_content.mdx"
    content = "# Introduction\n\nThis is the first sentence. This is another sentence.\n" \
              "## Chapter 1\n\nMore content here."
    file_path.write_text(content)
    
    # yield the directory path to the test function
    # test function ko directory path yield karo
    yield str(docs_dir)
    
    # Teardown: remove the directory after the test is done
    # Teardown: test complete hone ke baad directory ko remove karo
    shutil.rmtree(docs_dir)

@patch('backend.ingest.ingest_data.get_qdrant_client')
@patch('backend.ingest.ingest_data.get_gemini_embedding') # Updated embedding function name
@patch('backend.db.models.get_db') # Mock the get_db function
def test_ingest_data_success(mock_get_db, mock_get_gemini_embedding, mock_get_qdrant_client, temp_docs_dir):
    """
    Tests the entire ingestion process for a successful run, including Qdrant and Postgres storage.
    Poore ingestion process ko ek kamyab run ke liye test karta hai, jismein Qdrant aur Postgres storage bhi shamil hai.
    """
    # --- Mock Setup ---
    
    # Mock the embedding function to return a fixed-size vector
    # Embedding function ko mock karo taake woh ek fixed-size vector return kare
    mock_embedding_func = MagicMock()
    mock_embedding_func.embed_query.return_value = [0.1] * 10  # Dummy vector of size 10
    mock_get_gemini_embedding.return_value = mock_embedding_func
    
    # Mock the Qdrant client
    # Qdrant client ko mock karo
    mock_qdrant_client = MagicMock()
    mock_get_qdrant_client.return_value = mock_qdrant_client
    
    # Mock the SQLAlchemy session
    # SQLAlchemy session ko mock karo
    mock_session = MagicMock()
    mock_get_db.return_value.__enter__.return_value = mock_session # For context manager
    mock_get_db.return_value.__exit__.return_value = None # For context manager
    
    # --- Function Call ---
    
    collection_name = "test_collection"
    ingest_data(temp_docs_dir, collection_name)
    
    # --- Assertions ---
    
    # Assert that the Qdrant client was used to recreate the collection
    # Tasdeeq karein ke Qdrant client collection ko recreate karne ke liye istemal hua tha
    mock_qdrant_client.recreate_collection.assert_called_once()
    args, kwargs = mock_qdrant_client.recreate_collection.call_args
    assert kwargs['collection_name'] == collection_name
    assert kwargs['vectors_config'].size == 10  # Check if vector size was correctly detected

    # Assert that the Qdrant client's `upsert` method was called (changed from add to upsert)
    # Tasdeeq karein ke Qdrant client ka `upsert` method call hua tha
    mock_qdrant_client.upsert.assert_called_once()
    args, kwargs = mock_qdrant_client.upsert.call_args
    assert kwargs['collection_name'] == collection_name
    # Check that points were passed correctly and contain expected data
    # Check karein ke points sahi tarah se pass hue the aur unmein mutawaqqa data shamil hai
    assert len(kwargs['points']) > 0
    assert "This is the first sentence" in kwargs['points'][0].payload['text_content']
    assert "test_book_content.mdx" == kwargs['points'][0].payload['metadata']['source_file']

    # Assert that SQLAlchemy session methods were called for Postgres storage
    # Tasdeeq karein ke Postgres storage ke liye SQLAlchemy session methods call hue the
    assert mock_session.add.call_count == len(kwargs['points']) # Should add each chunk
    mock_session.commit.assert_called_once()
    mock_session.close.assert_called_once()

    # Verify that BookChunk objects were added with correct data
    # Tasdeeq karein ke BookChunk objects sahi data ke sath add hue the
    added_chunks = [call_arg.args[0] for call_arg in mock_session.add.call_args_list]
    assert added_chunks[0].text_content is not None
    assert added_chunks[0].metadata['source_file'] == "test_book_content.mdx"

