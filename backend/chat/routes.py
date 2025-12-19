"""
Chat history API routes.
Chat history ke liye API routes.
"""

from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy.orm import Session
from typing import Optional
from datetime import datetime

from db.models import get_db, ChatMessage, User
from auth.utils import get_current_user
from chat.schemas import (
    ChatHistoryResponse,
    ChatMessageResponse,
    SaveChatRequest,
    SaveChatResponse,
    DeleteChatResponse
)

router = APIRouter(prefix="/chat", tags=["chat"])


@router.get("/history", response_model=ChatHistoryResponse)
async def get_chat_history(
    page: int = Query(1, ge=1, description="Page number"),
    page_size: int = Query(20, ge=1, le=100, description="Items per page"),
    search: Optional[str] = Query(None, description="Search in questions and answers"),
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """
    Get chat history for the authenticated user with pagination and optional search.
    Authenticated user ke liye chat history pagination aur optional search ke saath hasil karo.
    """
    try:
        # Build query
        query = db.query(ChatMessage).filter(ChatMessage.user_id == current_user.id)

        # Add search filter if provided
        if search:
            search_pattern = f"%{search}%"
            query = query.filter(
                (ChatMessage.question.ilike(search_pattern)) |
                (ChatMessage.answer.ilike(search_pattern))
            )

        # Get total count
        total_count = query.count()

        # Apply pagination and ordering
        messages = query.order_by(ChatMessage.created_at.desc())\
            .offset((page - 1) * page_size)\
            .limit(page_size)\
            .all()

        # Convert to response models
        message_responses = [
            ChatMessageResponse(
                id=msg.id,
                question=msg.question,
                answer=msg.answer,
                source_chunks=msg.source_chunks,
                selected_text=msg.selected_text,
                created_at=msg.created_at
            )
            for msg in messages
        ]

        return ChatHistoryResponse(
            success=True,
            messages=message_responses,
            total_count=total_count,
            page=page,
            page_size=page_size
        )

    except Exception as e:
        print(f"Error fetching chat history: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to fetch chat history: {str(e)}")


@router.post("/save", response_model=SaveChatResponse)
async def save_chat_message(
    chat_data: SaveChatRequest,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """
    Save a chat message to history.
    Chat message ko history mein save karo.
    """
    try:
        # Create new chat message
        new_message = ChatMessage(
            user_id=current_user.id,
            question=chat_data.question,
            answer=chat_data.answer,
            source_chunks=chat_data.source_chunks,
            selected_text=chat_data.selected_text,
            created_at=datetime.utcnow()
        )

        db.add(new_message)
        db.commit()
        db.refresh(new_message)

        return SaveChatResponse(
            success=True,
            message="Chat message saved successfully",
            chat_id=new_message.id
        )

    except Exception as e:
        db.rollback()
        print(f"Error saving chat message: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to save chat message: {str(e)}")


@router.delete("/{chat_id}", response_model=DeleteChatResponse)
async def delete_chat_message(
    chat_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """
    Delete a chat message from history.
    Chat message ko history se delete karo.
    """
    try:
        # Find the message
        message = db.query(ChatMessage).filter(
            ChatMessage.id == chat_id,
            ChatMessage.user_id == current_user.id
        ).first()

        if not message:
            raise HTTPException(
                status_code=404,
                detail="Chat message not found or you don't have permission to delete it"
            )

        db.delete(message)
        db.commit()

        return DeleteChatResponse(
            success=True,
            message="Chat message deleted successfully"
        )

    except HTTPException:
        raise
    except Exception as e:
        db.rollback()
        print(f"Error deleting chat message: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to delete chat message: {str(e)}")


@router.get("/{chat_id}", response_model=ChatMessageResponse)
async def get_chat_message(
    chat_id: int,
    db: Session = Depends(get_db),
    current_user: User = Depends(get_current_user)
):
    """
    Get a specific chat message by ID.
    ID se specific chat message hasil karo.
    """
    try:
        message = db.query(ChatMessage).filter(
            ChatMessage.id == chat_id,
            ChatMessage.user_id == current_user.id
        ).first()

        if not message:
            raise HTTPException(
                status_code=404,
                detail="Chat message not found or you don't have permission to view it"
            )

        return ChatMessageResponse(
            id=message.id,
            question=message.question,
            answer=message.answer,
            source_chunks=message.source_chunks,
            selected_text=message.selected_text,
            created_at=message.created_at
        )

    except HTTPException:
        raise
    except Exception as e:
        print(f"Error fetching chat message: {str(e)}")
        raise HTTPException(status_code=500, detail=f"Failed to fetch chat message: {str(e)}")
