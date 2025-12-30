"""
API endpoints for session management in the RAG Chatbot
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from typing import Optional
from pydantic import BaseModel, field_validator

from ..database.config import get_async_session
from ..models.chat_session import ChatSession
from ..services.chat_session_service import chat_session_service


class CreateSessionRequest(BaseModel):
    user_id: Optional[str] = None

    @field_validator('user_id')
    @classmethod
    def validate_user_id(cls, v):
        if v is not None:
            if not v.strip():
                raise ValueError('User ID cannot be empty')
            if len(v) > 100:  # Limit user ID length
                raise ValueError('User ID cannot exceed 100 characters')
        return v


class CreateSessionResponse(BaseModel):
    session_id: str
    created_at: str


router = APIRouter(prefix="/sessions", tags=["sessions"])


@router.post("/", response_model=CreateSessionResponse)
async def create_session(
    request: CreateSessionRequest,
    db: AsyncSession = Depends(get_async_session)
):
    """
    Create a new chat session
    """
    try:
        # Create the session
        session = await chat_session_service.create_session(
            db=db,
            user_id=request.user_id
        )

        return CreateSessionResponse(
            session_id=str(session.id),
            created_at=session.created_at.isoformat()
        )

    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error creating session: {str(e)}"
        )


@router.get("/{session_id}", response_model=ChatSession)
async def get_session(
    session_id: str,
    db: AsyncSession = Depends(get_async_session)
):
    """
    Get session details by ID
    """
    try:
        session = await chat_session_service.get_session(db, session_id)
        if not session:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found"
            )

        return session

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving session: {str(e)}"
        )