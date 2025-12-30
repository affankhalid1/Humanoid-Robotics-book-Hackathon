from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete
from datetime import datetime, timedelta
from typing import Optional
import uuid
from ...models.chat_session import ChatSession
from ...config import settings


async def create_chat_session(db: AsyncSession, user_id: Optional[str] = None):
    """Create a new chat session"""
    expiry_time = datetime.utcnow() + timedelta(hours=settings.session_expiry_hours)

    db_session = ChatSession(
        user_id=user_id,
        expires_at=expiry_time
    )
    db.add(db_session)
    await db.commit()
    await db.refresh(db_session)
    return db_session


async def get_chat_session(db: AsyncSession, session_id: str):
    """Get a chat session by ID"""
    stmt = select(ChatSession).where(ChatSession.id == uuid.UUID(session_id))
    result = await db.execute(stmt)
    return result.scalar_one_or_none()


async def update_chat_session(db: AsyncSession, session_id: str, **kwargs):
    """Update a chat session"""
    stmt = update(ChatSession).where(ChatSession.id == uuid.UUID(session_id)).values(**kwargs)
    await db.execute(stmt)
    await db.commit()


async def delete_chat_session(db: AsyncSession, session_id: str):
    """Delete a chat session"""
    stmt = delete(ChatSession).where(ChatSession.id == uuid.UUID(session_id))
    await db.execute(stmt)
    await db.commit()