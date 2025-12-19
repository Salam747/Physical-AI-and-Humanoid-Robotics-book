# Authentication utility functions
# JWT token generation and password hashing
# JWT token generation aur password hashing ke liye utility functions

from jose import JWTError, jwt
from datetime import datetime, timedelta
import os
import bcrypt
from dotenv import load_dotenv

load_dotenv()

# Use bcrypt directly instead of passlib to avoid version conflicts

# JWT configuration from environment variables
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "fallback-secret-key-for-development")
ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
ACCESS_TOKEN_EXPIRE_HOURS = int(os.getenv("JWT_EXPIRY_HOURS", 24))


def hash_password(password: str) -> str:
    """
    Hash a plain text password using bcrypt.
    Plain text password ko bcrypt se hash karna.

    Args:
        password: Plain text password

    Returns:
        Hashed password string
    """
    # Convert password to bytes and hash it
    password_bytes = password.encode('utf-8')
    salt = bcrypt.gensalt()
    hashed = bcrypt.hashpw(password_bytes, salt)
    return hashed.decode('utf-8')


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a plain text password against a hashed password.
    Plain text password ko hashed password se verify karna.

    Args:
        plain_password: Plain text password to verify
        hashed_password: Hashed password from database

    Returns:
        True if password matches, False otherwise
    """
    # Convert both to bytes and verify
    password_bytes = plain_password.encode('utf-8')
    hashed_bytes = hashed_password.encode('utf-8')
    return bcrypt.checkpw(password_bytes, hashed_bytes)


def create_access_token(data: dict) -> str:
    """
    Create a JWT access token with expiration.
    Expiration ke saath JWT access token banana.

    Args:
        data: Dictionary containing user data to encode in token

    Returns:
        JWT token string
    """
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(hours=ACCESS_TOKEN_EXPIRE_HOURS)
    to_encode.update({
        "exp": expire,
        "iat": datetime.utcnow()
    })

    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> dict:
    """
    Verify and decode a JWT token.
    JWT token ko verify aur decode karna.

    Args:
        token: JWT token string

    Returns:
        Decoded token payload dict or None if invalid
    """
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except JWTError:
        return None


def get_user_from_token(token: str) -> dict:
    """
    Extract user information from JWT token.
    JWT token se user information nikalna.

    Args:
        token: JWT token string

    Returns:
        User data dict or None if invalid
    """
    payload = verify_token(token)
    if payload is None:
        return None

    # Extract user data from payload
    user_data = {
        "id": payload.get("sub"),
        "email": payload.get("email"),
        "first_name": payload.get("first_name"),
        "last_name": payload.get("last_name")
    }

    return user_data if all(user_data.values()) else None


# Dependency for FastAPI routes to get current user
from fastapi import Depends, HTTPException, Header
from sqlalchemy.orm import Session
from typing import Optional


def get_current_user(
    authorization: Optional[str] = Header(None),
    db: Session = Depends(None)  # Will be replaced with actual get_db in routes
):
    """
    FastAPI dependency to get current authenticated user from Authorization header.
    Authorization header se current authenticated user hasil karne ke liye FastAPI dependency.

    Args:
        authorization: Authorization header value
        db: Database session

    Returns:
        User object from database

    Raises:
        HTTPException: If token is invalid or user not found
    """
    if not authorization or not authorization.startswith("Bearer "):
        raise HTTPException(status_code=401, detail="Authorization header missing or invalid")

    token = authorization.replace("Bearer ", "")
    user_data = get_user_from_token(token)

    if not user_data:
        raise HTTPException(status_code=401, detail="Invalid or expired token")

    # Import here to avoid circular dependency
    from db.models import User, get_db

    # Get db session if not provided
    if db is None:
        db_gen = get_db()
        db = next(db_gen)
        try:
            user = db.query(User).filter(User.id == user_data["id"]).first()
            if not user:
                raise HTTPException(status_code=404, detail="User not found")
            return user
        finally:
            db.close()
    else:
        user = db.query(User).filter(User.id == user_data["id"]).first()
        if not user:
            raise HTTPException(status_code=404, detail="User not found")
        return user


