# Data Model: RAG Backend

**Date**: 2025-12-09
**Status**: Draft

This document describes the data entities required for the RAG backend. The primary data store for metadata will be a Neon serverless Postgres database, accessed via SQLAlchemy.

---

## Entity: `BookChunk`

This is the only entity required for the initial version of the backend. It represents a single chunk of text derived from the source `.mdx` book files.

### Description

A `BookChunk` stores the text content of a small piece of the book, along with metadata linking it back to its source file and its corresponding vector in the Qdrant database. This metadata is useful for debugging, citation, and potentially for filtering searches in the future.

### Schema

**Table Name**: `book_chunks`

| Column      | Type           | Constraints       | Description                                                                 |
|-------------|----------------|-------------------|-----------------------------------------------------------------------------|
| `id`        | `Integer`      | **Primary Key**   | A unique identifier for the chunk in the Postgres database.                 |
| `content`   | `Text`         | **Not Null**      | The actual text content of the chunk (typically 300-600 tokens).            |
| `source_file` | `String(255)`| **Not Null**      | The path of the source `.mdx` file from which this chunk was extracted.     |
| `vector_id` | `String(36)`   | **Not Null, Unique** | The UUID of the corresponding vector stored in the Qdrant collection.       |

### Example Record

```json
{
  "id": 101,
  "content": "ROS 2 is a flexible framework for writing robot software. It is a set of software libraries and tools that help you build robot applications.",
  "source_file": "module1/01-ros2-architecture.mdx",
  "vector_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef"
}
```

### Relationships

-   There are no relationships between tables in this simple, single-entity model. Each `BookChunk` is an independent record.
