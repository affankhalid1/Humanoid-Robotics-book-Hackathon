"""
Message Service for the RAG Chatbot
Handles creation, retrieval, and management of messages
"""

from typing import List, Optional
from sqlalchemy.ext.asyncio import AsyncSession
from datetime import datetime
from uuid import UUID
import uuid

from ..models.message import Message
from ..database.crud import (
    create_message, get_message, get_messages_by_session,
    update_message, delete_message
)
from ..utils import get_logger


class MessageService:
    """
    Service class for managing messages
    """
    def __init__(self):
        self.logger = get_logger(__name__)

    async def create_message(
        self,
        db: AsyncSession,
        session_id: str,
        role: str,
        content: str,
        metadata: Optional[dict] = None
    ) -> Message:
        """
        Create a new message

        Args:
            db: Database session
            session_id: Session UUID as string
            role: Message role ("user" or "assistant")
            content: Message content
            metadata: Optional metadata dictionary

        Returns:
            Created Message object
        """
        try:
            # Validate role
            if role not in ["user", "assistant"]:
                raise ValueError(f"Invalid role: {role}. Must be 'user' or 'assistant'")

            # Create the message
            message = await create_message(
                db=db,
                session_id=session_id,
                role=role,
                content=content,
                metadata=metadata or {}
            )

            self.logger.info(f"Created message {message.id} for session {session_id}")
            return message

        except Exception as e:
            self.logger.error(f"Error creating message for session {session_id}: {str(e)}")
            raise

    async def get_message(self, db: AsyncSession, message_id: str) -> Optional[Message]:
        """
        Retrieve a message by ID

        Args:
            db: Database session
            message_id: Message UUID as string

        Returns:
            Message object if found, None otherwise
        """
        try:
            message = await get_message(db, message_id)
            if message:
                self.logger.debug(f"Retrieved message: {message.id}")
            else:
                self.logger.warning(f"Message not found: {message_id}")

            return message

        except Exception as e:
            self.logger.error(f"Error retrieving message {message_id}: {str(e)}")
            raise

    async def get_messages_by_session(self, db: AsyncSession, session_id: str) -> List[Message]:
        """
        Retrieve all messages for a session

        Args:
            db: Database session
            session_id: Session UUID as string

        Returns:
            List of Message objects
        """
        try:
            messages = await get_messages_by_session(db, session_id)
            self.logger.debug(f"Retrieved {len(messages)} messages for session {session_id}")
            return messages

        except Exception as e:
            self.logger.error(f"Error retrieving messages for session {session_id}: {str(e)}")
            raise

    async def create_user_message(self, db: AsyncSession, session_id: str, content: str) -> Message:
        """
        Create a user message

        Args:
            db: Database session
            session_id: Session UUID as string
            content: User message content

        Returns:
            Created Message object
        """
        return await self.create_message(db, session_id, "user", content)

    async def create_assistant_message(self, db: AsyncSession, session_id: str, content: str) -> Message:
        """
        Create an assistant message

        Args:
            db: Database session
            session_id: Session UUID as string
            content: Assistant message content

        Returns:
            Created Message object
        """
        return await self.create_message(db, session_id, "assistant", content)


# Global instance for easy access
message_service = MessageService()