#!/usr/bin/env python3
"""
Qdrant vector loader for the RAG Chatbot.
Loads embedded content chunks into Qdrant vector store.
"""

import asyncio
import json
from typing import List, Dict, Any
from pathlib import Path
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid


class QdrantLoader:
    """
    Loader for storing embedded content chunks in Qdrant.
    """

    def __init__(self, host: str = "localhost", port: int = 6333, api_key: str = None, collection_name: str = "book_content"):
        self.client = QdrantClient(
            host=host,
            port=port,
            api_key=api_key
        )
        self.collection_name = collection_name

    async def initialize_collection(self):
        """
        Initialize the collection with proper configuration.
        """
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with appropriate configuration
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1536,  # Default size, adjust based on embedding model
                        distance=models.Distance.COSINE
                    )
                )

                # Create payload index for faster filtering
                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="content_type",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="chapter_name",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="module_path",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                print(f"Created collection: {self.collection_name}")
            else:
                print(f"Collection {self.collection_name} already exists")

        except Exception as e:
            print(f"Error initializing collection: {str(e)}")
            raise

    async def load_chunks(self, chunks: List[Dict[str, Any]]):
        """
        Load embedded content chunks into Qdrant.

        Args:
            chunks: List of embedded content chunks
        """
        points = []
        for chunk in chunks:
            # Generate a unique ID for the point
            point_id = str(uuid.uuid4())

            # Create a Qdrant point
            point = models.PointStruct(
                id=point_id,
                vector=chunk.get('embedding', []),
                payload={
                    "content": chunk.get('content', ''),
                    "content_type": chunk.get('content_type', 'prose'),
                    "chapter_name": chunk.get('chapter_name', ''),
                    "module_path": chunk.get('module_path', ''),
                    "source_file": chunk.get('source_file', ''),
                    "source_line_start": chunk.get('source_line_start', 0),
                    "source_line_end": chunk.get('source_line_end', 0),
                    "metadata": chunk.get('metadata', {})
                }
            )
            points.append(point)

        # Upload points to Qdrant in batches
        batch_size = 100
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            await self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )
            print(f"Uploaded batch {i // batch_size + 1}/{(len(points) - 1) // batch_size + 1}")

        print(f"Successfully loaded {len(chunks)} content chunks to Qdrant")

    async def load_from_file(self, input_file: str):
        """
        Load embedded content chunks from a JSON file into Qdrant.

        Args:
            input_file: Path to the JSON file with embedded chunks
        """
        # Load embedded chunks
        with open(input_file, 'r', encoding='utf-8') as f:
            chunks = json.load(f)

        # Initialize collection
        await self.initialize_collection()

        # Load chunks to Qdrant
        await self.load_chunks(chunks)


def main():
    """
    Main function to run the Qdrant loader.
    """
    import argparse

    parser = argparse.ArgumentParser(description='Load embedded content to Qdrant for RAG Chatbot')
    parser.add_argument('--chunks-path', required=True, help='Path to JSON file with embedded content chunks')
    parser.add_argument('--host', default='localhost', help='Qdrant host')
    parser.add_argument('--port', type=int, default=6333, help='Qdrant port')
    parser.add_argument('--api-key', help='Qdrant API key')
    parser.add_argument('--collection-name', default='book_content', help='Qdrant collection name')

    args = parser.parse_args()

    # Initialize loader
    loader = QdrantLoader(
        host=args.host,
        port=args.port,
        api_key=args.api_key,
        collection_name=args.collection_name
    )

    # Load chunks to Qdrant
    asyncio.run(loader.load_from_file(args.chunks_path))

    print("Content loading completed successfully!")


if __name__ == "__main__":
    main()