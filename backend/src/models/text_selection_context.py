from sqlalchemy import Column, String, DateTime, Text, ForeignKey
from sqlalchemy.dialects.postgresql import UUID
from datetime import datetime
import uuid
from ..database.config import Base


class TextSelectionContext(Base):
    __tablename__ = "text_selection_contexts"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    session_id = Column(UUID(as_uuid=True), ForeignKey("chat_sessions.id"), nullable=False)
    selected_text = Column(Text, nullable=False)
    page_url = Column(String, nullable=True)
    page_title = Column(String, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)

    def __repr__(self):
        return f"<TextSelectionContext(id={self.id}, session_id={self.session_id})>"