# Data Model: RAG Chatbot for Physical AI Humanoid Robotics Book

**Feature**: RAG Chatbot Integration
**Date**: 2025-12-25
**Status**: Design

## Overview

This document defines the data models for the RAG chatbot system, based on the key entities identified in the feature specification. These models will be implemented in both the backend (Python) and frontend (JavaScript/TypeScript) with appropriate validation rules.

## Core Entities

### 1. Chat Session

**Description**: Represents a user's conversation with the chatbot, including question history, user context, and session metadata

**Fields**:
- `id` (UUID): Unique identifier for the session
- `user_id` (String): Identifier for the user (optional for anonymous sessions)
- `created_at` (DateTime): Timestamp when session was created
- `updated_at` (DateTime): Timestamp of last activity
- `expires_at` (DateTime): Session expiration time
- `metadata` (JSON): Additional session information

**Validation Rules**:
- `id` must be a valid UUID
- `created_at` must be before `updated_at`
- `expires_at` must be in the future
- `user_id` can be null for anonymous sessions

**Relationships**:
- One-to-many with Message entities
- One-to-many with CitationReference entities

### 2. Message

**Description**: Represents a single message in the chat session (either user question or bot response)

**Fields**:
- `id` (UUID): Unique identifier for the message
- `session_id` (UUID): Reference to the parent chat session
- `role` (String): Either "user" or "assistant"
- `content` (Text): The message content
- `timestamp` (DateTime): When the message was created
- `metadata` (JSON): Additional message information (e.g., source citations)

**Validation Rules**:
- `id` must be a valid UUID
- `session_id` must reference an existing ChatSession
- `role` must be either "user" or "assistant"
- `content` must not be empty

**Relationships**:
- Many-to-one with ChatSession
- One-to-many with CitationReference entities

### 3. Knowledge Chunk

**Description**: Represents a segment of book content (prose or code) that has been processed and embedded for RAG retrieval

**Fields**:
- `id` (UUID): Unique identifier for the chunk
- `content` (Text): The actual content text
- `content_type` (String): Either "prose" or "code"
- `chapter_name` (String): Name of the chapter this content belongs to
- `module_path` (String): Path to the module if this is code content
- `source_file` (String): Original file path
- `source_line_start` (Integer): Starting line number in source file
- `source_line_end` (Integer): Ending line number in source file
- `embedding_vector` (Binary): The vector representation of the content
- `created_at` (DateTime): When this chunk was created
- `metadata` (JSON): Additional metadata for the chunk

**Validation Rules**:
- `id` must be a valid UUID
- `content_type` must be either "prose" or "code"
- `content` must not be empty
- `source_line_start` must be less than or equal to `source_line_end`
- `embedding_vector` must be properly formatted

**Relationships**:
- One-to-many with CitationReference entities

### 4. Citation Reference

**Description**: Represents a link between a chatbot response and the specific source content in the book

**Fields**:
- `id` (UUID): Unique identifier for the citation
- `session_id` (UUID): Reference to the chat session
- `message_id` (UUID): Reference to the message containing the citation
- `knowledge_chunk_id` (UUID): Reference to the source content
- `relevance_score` (Float): Score indicating how relevant the chunk was to the response (0.0-1.0)
- `created_at` (DateTime): When the citation was created

**Validation Rules**:
- `id` must be a valid UUID
- `session_id` must reference an existing ChatSession
- `message_id` must reference an existing Message
- `knowledge_chunk_id` must reference an existing KnowledgeChunk
- `relevance_score` must be between 0.0 and 1.0

**Relationships**:
- Many-to-one with ChatSession
- Many-to-one with Message
- Many-to-one with KnowledgeChunk

### 5. Text Selection Context

**Description**: Represents the user-selected text on a page that influences the chatbot's response focus

**Fields**:
- `id` (UUID): Unique identifier for the selection context
- `session_id` (UUID): Reference to the chat session
- `selected_text` (Text): The text that was selected by the user
- `page_url` (String): URL of the page where text was selected
- `page_title` (String): Title of the page where text was selected
- `created_at` (DateTime): When the selection was made

**Validation Rules**:
- `id` must be a valid UUID
- `session_id` must reference an existing ChatSession
- `selected_text` must not be empty
- `page_url` must be a valid URL

**Relationships**:
- Many-to-one with ChatSession

## API Contracts

### 1. Chat Session Management

**Endpoint**: `POST /api/sessions`
**Description**: Create a new chat session
**Request**:
```json
{
  "user_id": "string (optional)"
}
```
**Response**:
```json
{
  "session_id": "uuid",
  "created_at": "datetime"
}
```

### 2. Send Message

**Endpoint**: `POST /api/sessions/{session_id}/messages`
**Description**: Send a message to the chatbot
**Request**:
```json
{
  "content": "string",
  "selected_text_context": "string (optional)"
}
```
**Response**:
```json
{
  "message_id": "uuid",
  "response": "string",
  "citations": [
    {
      "chunk_id": "uuid",
      "chapter_name": "string",
      "module_path": "string",
      "relevance_score": "float"
    }
  ]
}
```

### 3. Retrieve Session History

**Endpoint**: `GET /api/sessions/{session_id}/messages`
**Description**: Get the message history for a session
**Response**:
```json
{
  "messages": [
    {
      "id": "uuid",
      "role": "string",
      "content": "string",
      "timestamp": "datetime"
    }
  ]
}
```

## Database Schema

### Tables

1. `chat_sessions` - Stores chat session information
2. `messages` - Stores individual messages in sessions
3. `knowledge_chunks` - Stores embedded content chunks
4. `citation_references` - Stores citation links between messages and content
5. `text_selection_contexts` - Stores user text selections

### Indexes

1. Index on `chat_sessions.expires_at` for efficient cleanup
2. Index on `messages.session_id` for session-based queries
3. Index on `citation_references.session_id` and `citation_references.message_id`
4. Index on `knowledge_chunks.content_type` and `knowledge_chunks.chapter_name` for content-based queries