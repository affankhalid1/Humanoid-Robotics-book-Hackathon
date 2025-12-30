import logging
import sys
from typing import Any, Dict
from ..config import settings


def get_logger(name: str, level: int = None) -> logging.Logger:
    """
    Create and configure a logger with the specified name

    Args:
        name: Name of the logger
        level: Logging level (defaults to DEBUG in development, INFO in production)

    Returns:
        Configured logger instance
    """
    if level is None:
        level = logging.DEBUG if settings.app_debug else logging.INFO

    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Avoid adding multiple handlers to the same logger
    if logger.handlers:
        return logger

    # Create console handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)

    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    handler.setFormatter(formatter)

    # Add handler to logger
    logger.addHandler(handler)

    return logger