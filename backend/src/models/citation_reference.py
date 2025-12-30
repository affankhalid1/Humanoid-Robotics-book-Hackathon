from sqlalchemy import Column, String, DateTime, Float, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from ..database.config import Base


class CitationReference(Base):
    __tablename__ = "citation_references"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    message_id = Column(UUID(as_uuid=True), ForeignKey("messages.id"), nullable=False)
    knowledge_chunk_id = Column(UUID(as_uuid=True), ForeignKey("knowledge_chunks.id"), nullable=False)
    relevance_score = Column(Float, nullable=False)  # 0.0 to 1.0
    created_at = Column(DateTime, default=datetime.utcnow)

    def __repr__(self):
        return f"<CitationReference(id={self.id}, message_id={self.message_id}, chunk_id={self.knowledge_chunk_id})>"