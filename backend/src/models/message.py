from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.mutable import MutableDict
from sqlalchemy.dialects.postgresql import JSONB
from sqlalchemy.orm import relationship
from datetime import datetime
import uuid
from ..database.config import Base


class Message(Base):
    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    role = Column(String, nullable=False)  # "user" or "assistant"
    content = Column(Text, nullable=False)
    timestamp = Column(DateTime, default=datetime.utcnow)
    metadata = Column(MutableDict.as_mutable(JSONB), default={})

    # Relationship
    session = relationship("ChatSession", back_populates="messages")

    def __repr__(self):
        return f"<Message(id={self.id}, role={self.role}, session_id={self.session_id})>"


# Add relationship to ChatSession
ChatSession.messages = relationship("Message", order_by=Message.timestamp, back_populates="session")