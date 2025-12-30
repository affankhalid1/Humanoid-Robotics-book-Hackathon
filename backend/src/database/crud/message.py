from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from sqlalchemy import update, delete
from datetime import datetime
from typing import List
import uuid
from ...models.message import Message


async def create_message(db: AsyncSession, session_id: str, role: str, content: str, metadata: dict = None):
    """Create a new message"""
    if metadata is None:
        metadata = {}

    db_message = Message(
        session_id=uuid.UUID(session_id),
        role=role,
        content=content,
        metadata=metadata
    )
    db.add(db_message)
    await db.commit()
    await db.refresh(db_message)
    return db_message


async def get_message(db: AsyncSession, message_id: str):
    """Get a message by ID"""
    stmt = select(Message).where(Message.id == uuid.UUID(message_id))
    result = await db.execute(stmt)
    return result.scalar_one_or_none()


async def get_messages_by_session(db: AsyncSession, session_id: str) -> List[Message]:
    """Get all messages for a session"""
    stmt = select(Message).where(Message.session_id == uuid.UUID(session_id)).order_by(Message.timestamp)
    result = await db.execute(stmt)
    return result.scalars().all()


async def update_message(db: AsyncSession, message_id: str, **kwargs):
    """Update a message"""
    stmt = update(Message).where(Message.id == uuid.UUID(message_id)).values(**kwargs)
    await db.execute(stmt)
    await db.commit()


async def delete_message(db: AsyncSession, message_id: str):
    """Delete a message"""
    stmt = delete(Message).where(Message.id == uuid.UUID(message_id))
    await db.execute(stmt)
    await db.commit()