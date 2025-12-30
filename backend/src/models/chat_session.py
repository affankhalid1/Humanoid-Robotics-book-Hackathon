from sqlalchemy import Column, String, DateTime, Text, Boolean
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.mutable import MutableDict
from sqlalchemy.dialects.postgresql import JSONB
from datetime import datetime
import uuid
from ..database.config import Base


class ChatSession(Base):
    __tablename__ = "chat_sessions"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(String, nullable=True)  # Optional for anonymous sessions
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    expires_at = Column(DateTime)
    metadata = Column(MutableDict.as_mutable(JSONB), default={})
    is_active = Column(Boolean, default=True)

    def __repr__(self):
        return f"<ChatSession(id={self.id}, user_id={self.user_id})>"