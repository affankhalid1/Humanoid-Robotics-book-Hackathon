from sqlalchemy import Column, String, DateTime, Text, Integer
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.mutable import MutableDict
from sqlalchemy.dialects.postgresql import JSONB
from datetime import datetime
import uuid
from ..database.config import Base


class KnowledgeChunk(Base):
    __tablename__ = "knowledge_chunks"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    content = Column(Text, nullable=False)
    content_type = Column(String, nullable=False)  # "prose" or "code"
    chapter_name = Column(String, nullable=True)
    module_path = Column(String, nullable=True)
    source_file = Column(String, nullable=True)
    source_line_start = Column(Integer, nullable=True)
    source_line_end = Column(Integer, nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    metadata = Column(MutableDict.as_mutable(JSONB), default={})

    def __repr__(self):
        return f"<KnowledgeChunk(id={self.id}, content_type={self.content_type}, chapter={self.chapter_name})>"