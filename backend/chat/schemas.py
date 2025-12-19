"""
Pydantic schemas for chat history endpoints.
Chat history endpoints ke liye Pydantic schemas.
"""

from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
from datetime import datetime


class ChatMessageResponse(BaseModel):
    """Response model for a single chat message"""
    id: int
    question: str
    answer: str
    source_chunks: Optional[List[Dict[str, Any]]] = None
    selected_text: Optional[str] = None
    created_at: datetime

    class Config:
        from_attributes = True


class ChatHistoryResponse(BaseModel):
    """Response model for chat history list"""
    success: bool
    messages: List[ChatMessageResponse]
    total_count: int
    page: int
    page_size: int


class SaveChatRequest(BaseModel):
    """Request model for saving a chat message"""
    question: str = Field(..., min_length=1, max_length=5000)
    answer: str = Field(..., min_length=1)
    source_chunks: Optional[List[Dict[str, Any]]] = None
    selected_text: Optional[str] = None


class SaveChatResponse(BaseModel):
    """Response model for saving a chat message"""
    success: bool
    message: str
    chat_id: Optional[int] = None


class DeleteChatResponse(BaseModel):
    """Response model for deleting a chat message"""
    success: bool
    message: str
