"""
Chat Session Management Service for the RAG Chatbot
Handles creation, retrieval, and management of chat sessions
"""

from typing import Optional
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime, timedelta
from uuid import UUID
import uuid

from ..models.chat_session import ChatSession
from ..database.crud import create_chat_session, get_chat_session
from ..config import settings
from ..utils import get_logger


class ChatSessionService:
    """
    Service class for managing chat sessions
    """
    def __init__(self):
        self.logger = get_logger(__name__)

    async def create_session(self, db: AsyncSession, user_id: Optional[str] = None) -> ChatSession:
        """
        Create a new chat session

        Args:
            db: Database session
            user_id: Optional user identifier

        Returns:
            Created ChatSession object
        """
        try:
            # Calculate session expiry time
            expiry_time = datetime.utcnow() + timedelta(hours=settings.session_expiry_hours)

            # Create the session
            session = await create_chat_session(
                db=db,
                user_id=user_id
            )

            self.logger.info(f"Created new chat session: {session.id}")
            return session

        except Exception as e:
            self.logger.error(f"Error creating chat session: {str(e)}")
            raise

    async def get_session(self, db: AsyncSession, session_id: str) -> Optional[ChatSession]:
        """
        Retrieve a chat session by ID

        Args:
            db: Database session
            session_id: Session UUID as string

        Returns:
            ChatSession object if found, None otherwise
        """
        try:
            session = await get_chat_session(db, session_id)
            if session:
                self.logger.debug(f"Retrieved chat session: {session.id}")
            else:
                self.logger.warning(f"Chat session not found: {session_id}")

            return session

        except Exception as e:
            self.logger.error(f"Error retrieving chat session {session_id}: {str(e)}")
            raise

    async def validate_session(self, db: AsyncSession, session_id: str) -> bool:
        """
        Validate if a session is active and not expired

        Args:
            db: Database session
            session_id: Session UUID as string

        Returns:
            True if session is valid, False otherwise
        """
        try:
            session = await self.get_session(db, session_id)
            if not session:
                return False

            # Check if session is active and not expired
            if not session.is_active:
                return False

            if session.expires_at and session.expires_at < datetime.utcnow():
                # Mark session as inactive if expired
                await self._mark_session_inactive(db, session.id)
                return False

            return True

        except Exception as e:
            self.logger.error(f"Error validating session {session_id}: {str(e)}")
            return False

    async def _mark_session_inactive(self, db: AsyncSession, session_id: UUID):
        """
        Mark a session as inactive

        Args:
            db: Database session
            session_id: Session UUID
        """
        try:
            from ..database.crud import update_chat_session
            await update_chat_session(db, str(session_id), is_active=False)
            self.logger.info(f"Marked session as inactive: {session_id}")
        except Exception as e:
            self.logger.error(f"Error marking session as inactive {session_id}: {str(e)}")


# Global instance for easy access
chat_session_service = ChatSessionService()