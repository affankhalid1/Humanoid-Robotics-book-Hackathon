"""
API endpoints for message management in the RAG Chatbot
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List, Optional
from pydantic import BaseModel

from ..database.config import get_async_session
from ..models.message import Message
from ..models.citation_reference import CitationReference
from ..services.message_service import message_service
from ..services.rag_service import rag_service
from ..services.llm_service import llm_service
from ..services.citation_service import citation_service


from pydantic import BaseModel, field_validator


class CreateMessageRequest(BaseModel):
    content: str
    selected_text_context: Optional[str] = None

    @field_validator('content')
    @classmethod
    def validate_content(cls, v):
        if not v or not v.strip():
            raise ValueError('Content cannot be empty')
        if len(v.strip()) < 1:
            raise ValueError('Content must be at least 1 character long')
        if len(v) > 5000:  # Limit content length
            raise ValueError('Content cannot exceed 5000 characters')
        return v.strip()

    @field_validator('selected_text_context')
    @classmethod
    def validate_selected_text_context(cls, v):
        if v is not None and len(v) > 10000:  # Limit selected text context length
            raise ValueError('Selected text context cannot exceed 10000 characters')
        return v


class CitationResponse(BaseModel):
    citation_id: str
    knowledge_chunk_id: str
    chapter_name: str
    module_path: str
    relevance_score: float
    source_file: str
    content_type: str
    source_line_start: Optional[int] = None
    source_line_end: Optional[int] = None
    source_url: str = ""
    source_text: str = ""


class CreateMessageResponse(BaseModel):
    message_id: str
    response: str
    citations: List[CitationResponse]


class MessageResponse(BaseModel):
    id: str
    role: str
    content: str
    timestamp: str


router = APIRouter(prefix="/sessions/{session_id}/messages", tags=["messages"])


@router.post("/", response_model=CreateMessageResponse)
async def create_message(
    session_id: str,
    request: CreateMessageRequest,
    db: AsyncSession = Depends(get_async_session)
):
    """
    Send a message to the chatbot and receive a response
    """
    try:
        # Validate the session
        is_valid = await message_service.chat_session_service.validate_session(db, session_id)
        if not is_valid:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found or expired"
            )

        # Create user message
        user_message = await message_service.create_user_message(
            db=db,
            session_id=session_id,
            content=request.content
        )

        # Retrieve relevant content using RAG, considering selected text context
        relevant_content = await rag_service.retrieve_relevant_content(
            query=request.content,
            limit=5,
            selected_text_context=request.selected_text_context
        )

        # Generate response using LLM with context
        response_data = await llm_service.generate_response_with_citations(
            prompt=request.content,
            context=relevant_content,
            selected_text_context=request.selected_text_context
        )

        # Create assistant message
        assistant_message = await message_service.create_assistant_message(
            db=db,
            session_id=session_id,
            content=response_data["response"]
        )

        # Create citation references with enhanced information
        citations = []
        for citation in response_data.get("citations", []):
            # Create citation reference with enhanced information in database
            citation_info = await citation_service.create_citation_with_chunk_info(
                db=db,
                session_id=session_id,
                message_id=str(assistant_message.id),
                knowledge_chunk_id=citation.get("chunk_id", ""),
                relevance_score=citation.get("relevance_score", 0.0)
            )
            citations.append(CitationResponse(
                citation_id=citation_info["citation_id"],
                knowledge_chunk_id=citation_info["knowledge_chunk_id"],
                chapter_name=citation_info["chapter_name"],
                module_path=citation_info["module_path"],
                relevance_score=citation_info["relevance_score"],
                source_file=citation_info["source_file"],
                content_type=citation_info["content_type"],
                source_line_start=citation_info["source_line_start"],
                source_line_end=citation_info["source_line_end"]
            ))

        return CreateMessageResponse(
            message_id=str(assistant_message.id),
            response=response_data["response"],
            citations=citations
        )

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error processing message: {str(e)}"
        )


@router.get("/", response_model=List[MessageResponse])
async def get_messages(
    session_id: str,
    db: AsyncSession = Depends(get_async_session)
):
    """
    Get message history for a session
    """
    try:
        # Validate the session
        is_valid = await message_service.chat_session_service.validate_session(db, session_id)
        if not is_valid:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Session not found or expired"
            )

        # Get messages for the session
        messages = await message_service.get_messages_by_session(db, session_id)

        # Format response
        formatted_messages = []
        for msg in messages:
            formatted_messages.append(MessageResponse(
                id=str(msg.id),
                role=msg.role,
                content=msg.content,
                timestamp=msg.timestamp.isoformat()
            ))

        return formatted_messages

    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Error retrieving messages: {str(e)}"
        )