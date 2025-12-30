from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete
from datetime import datetime
from typing import List, Optional
import uuid
from ...models.knowledge_chunk import KnowledgeChunk


async def create_knowledge_chunk(
    db: AsyncSession,
    content: str,
    content_type: str,
    chapter_name: Optional[str] = None,
    module_path: Optional[str] = None,
    source_file: Optional[str] = None,
    source_line_start: Optional[int] = None,
    source_line_end: Optional[int] = None,
    metadata: dict = None
):
    """Create a new knowledge chunk"""
    if metadata is None:
        metadata = {}

    db_chunk = KnowledgeChunk(
        content=content,
        content_type=content_type,
        chapter_name=chapter_name,
        module_path=module_path,
        source_file=source_file,
        source_line_start=source_line_start,
        source_line_end=source_line_end,
        metadata=metadata
    )
    db.add(db_chunk)
    await db.commit()
    await db.refresh(db_chunk)
    return db_chunk


async def get_knowledge_chunk(db: AsyncSession, chunk_id: str):
    """Get a knowledge chunk by ID"""
    stmt = select(KnowledgeChunk).where(KnowledgeChunk.id == uuid.UUID(chunk_id))
    result = await db.execute(stmt)
    return result.scalar_one_or_none()


async def get_knowledge_chunks(db: AsyncSession, skip: int = 0, limit: int = 100) -> List[KnowledgeChunk]:
    """Get knowledge chunks with pagination"""
    stmt = select(KnowledgeChunk).offset(skip).limit(limit)
    result = await db.execute(stmt)
    return result.scalars().all()


async def update_knowledge_chunk(db: AsyncSession, chunk_id: str, **kwargs):
    """Update a knowledge chunk"""
    stmt = update(KnowledgeChunk).where(KnowledgeChunk.id == uuid.UUID(chunk_id)).values(**kwargs)
    await db.execute(stmt)
    await db.commit()


async def delete_knowledge_chunk(db: AsyncSession, chunk_id: str):
    """Delete a knowledge chunk"""
    stmt = delete(KnowledgeChunk).where(KnowledgeChunk.id == uuid.UUID(chunk_id))
    await db.execute(stmt)
    await db.commit()