from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete
from datetime import datetime
from typing import Optional
import uuid
from ...models.text_selection_context import TextSelectionContext


async def create_text_selection_context(
    db: AsyncSession,
    session_id: str,
    selected_text: str,
    page_url: Optional[str] = None,
    page_title: Optional[str] = None
):
    """Create a new text selection context"""
    db_context = TextSelectionContext(
        session_id=uuid.UUID(session_id),
        selected_text=selected_text,
        page_url=page_url,
        page_title=page_title
    )
    db.add(db_context)
    await db.commit()
    await db.refresh(db_context)
    return db_context


async def get_text_selection_context(db: AsyncSession, context_id: str):
    """Get a text selection context by ID"""
    stmt = select(TextSelectionContext).where(TextSelectionContext.id == uuid.UUID(context_id))
    result = await db.execute(stmt)
    return result.scalar_one_or_none()


async def update_text_selection_context(db: AsyncSession, context_id: str, **kwargs):
    """Update a text selection context"""
    stmt = update(TextSelectionContext).where(TextSelectionContext.id == uuid.UUID(context_id)).values(**kwargs)
    await db.execute(stmt)
    await db.commit()


async def delete_text_selection_context(db: AsyncSession, context_id: str):
    """Delete a text selection context"""
    stmt = delete(TextSelectionContext).where(TextSelectionContext.id == uuid.UUID(context_id))
    await db.execute(stmt)
    await db.commit()