from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from uuid import uuid4
import logging
from ..config import settings
from ..utils import get_logger


class QdrantClientWrapper:
    """
    Wrapper class for Qdrant client to handle vector storage operations
    """
    def __init__(self):
        self.client = QdrantClient(
            host=settings.qdrant_host,
            port=settings.qdrant_port,
            api_key=settings.qdrant_api_key
        )
        self.collection_name = settings.qdrant_collection_name
        self.logger = get_logger(__name__)

    async def initialize_collection(self):
        """
        Initialize the collection with proper configuration
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
                        size=1536,  # Default size, will be adjusted based on embedding model
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

                self.logger.info(f"Created collection: {self.collection_name}")
            else:
                self.logger.info(f"Collection {self.collection_name} already exists")

        except Exception as e:
            self.logger.error(f"Error initializing collection: {str(e)}")
            raise

    async def store_embedding(self, content: str, embedding: List[float], metadata: Dict[str, Any]) -> str:
        """
        Store a content chunk with its embedding in Qdrant

        Args:
            content: The original content text
            embedding: The vector embedding
            metadata: Additional metadata to store with the vector

        Returns:
            The ID of the stored point
        """
        point_id = str(uuid4())

        try:
            await self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=point_id,
                        vector=embedding,
                        payload={
                            "content": content,
                            **metadata
                        }
                    )
                ]
            )

            self.logger.debug(f"Stored embedding with ID: {point_id}")
            return point_id

        except Exception as e:
            self.logger.error(f"Error storing embedding: {str(e)}")
            raise

    async def search_similar(self, query_embedding: List[float], limit: int = 10, filters: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Search for similar content based on embedding

        Args:
            query_embedding: The query vector to search for
            limit: Maximum number of results to return
            filters: Optional filters to apply to the search

        Returns:
            List of similar content with metadata and similarity scores
        """
        try:
            search_filter = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    if isinstance(value, list):
                        # Handle list of values (OR condition)
                        conditions = [models.FieldCondition(key=key, match=models.MatchValue(value=v)) for v in value]
                        filter_conditions.append(models.Filter(must=conditions))
                    else:
                        # Handle single value
                        filter_conditions.append(
                            models.FieldCondition(key=key, match=models.MatchValue(value=value))
                        )

                if filter_conditions:
                    search_filter = models.Filter(must=filter_conditions)

            results = await self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                query_filter=search_filter,
                with_payload=True
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "metadata": {k: v for k, v in result.payload.items() if k != "content"},
                    "score": result.score
                })

            self.logger.debug(f"Found {len(formatted_results)} similar results")
            return formatted_results

        except Exception as e:
            self.logger.error(f"Error searching for similar content: {str(e)}")
            raise

    async def delete_by_id(self, point_id: str):
        """
        Delete a point by its ID

        Args:
            point_id: The ID of the point to delete
        """
        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[point_id])
            )

            self.logger.debug(f"Deleted point with ID: {point_id}")

        except Exception as e:
            self.logger.error(f"Error deleting point {point_id}: {str(e)}")
            raise

    async def get_by_id(self, point_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a point by its ID

        Args:
            point_id: The ID of the point to retrieve

        Returns:
            The point data or None if not found
        """
        try:
            points = await self.client.retrieve(
                collection_name=self.collection_name,
                ids=[point_id],
                with_payload=True
            )

            if points:
                point = points[0]
                return {
                    "id": point.id,
                    "content": point.payload.get("content", ""),
                    "metadata": {k: v for k, v in point.payload.items() if k != "content"}
                }

            return None

        except Exception as e:
            self.logger.error(f"Error retrieving point {point_id}: {str(e)}")
            raise