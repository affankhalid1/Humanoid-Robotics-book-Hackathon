from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete
from datetime import datetime
from typing import List
import uuid
from ...models.citation_reference import CitationReference


async def create_citation_reference(
    db: AsyncSession,
    session_id: str,
    message_id: str,
    knowledge_chunk_id: str,
    relevance_score: float
):
    """Create a new citation reference"""
    db_citation = CitationReference(
        session_id=uuid.UUID(session_id),
        message_id=uuid.UUID(message_id),
        knowledge_chunk_id=uuid.UUID(knowledge_chunk_id),
        relevance_score=relevance_score
    )
    db.add(db_citation)
    await db.commit()
    await db.refresh(db_citation)
    return db_citation


async def get_citation_reference(db: AsyncSession, citation_id: str):
    """Get a citation reference by ID"""
    stmt = select(CitationReference).where(CitationReference.id == uuid.UUID(citation_id))
    result = await db.execute(stmt)
    return result.scalar_one_or_none()


async def get_citations_by_message(db: AsyncSession, message_id: str) -> List[CitationReference]:
    """Get all citations for a message"""
    stmt = select(CitationReference).where(CitationReference.message_id == uuid.UUID(message_id))
    result = await db.execute(stmt)
    return result.scalars().all()


async def update_citation_reference(db: AsyncSession, citation_id: str, **kwargs):
    """Update a citation reference"""
    stmt = update(CitationReference).where(CitationReference.id == uuid.UUID(citation_id)).values(**kwargs)
    await db.execute(stmt)
    await db.commit()


async def delete_citation_reference(db: AsyncSession, citation_id: str):
    """Delete a citation reference"""
    stmt = delete(CitationReference).where(CitationReference.id == uuid.UUID(citation_id))
    await db.execute(stmt)
    await db.commit()