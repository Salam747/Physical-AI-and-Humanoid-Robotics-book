# Murad... Asking for brother's help...
# This file defines our database tables using SQLAlchemy.
# Yeh file SQLAlchemy ka istemal karke hamare database tables ko define karti hai.

from sqlalchemy import create_engine, Column, Integer, String, Text, DateTime
from sqlalchemy.dialects.postgresql import JSONB # For PostgreSQL JSONB type
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import os
import uuid # For generating UUIDs for the id field
from dotenv import load_dotenv

load_dotenv()

# Get the database URL from environment variables
# Environment variables se database URL hasil karo
DATABASE_URL = os.getenv("DATABASE_URL") 

if not DATABASE_URL:
    print("WARNING: DATABASE_URL not found. Using in-memory SQLite database for now.")
    # Fallback to an in-memory SQLite database if Neon URL is not set
    # Agar Neon URL set nahi hai to in-memory SQLite database ka istemal karo
    DATABASE_URL = "sqlite:///:memory:"

# The Base which our model will inherit
# Woh Base jisse hamara model inherit karega
Base = declarative_base()

class BookChunk(Base):
    """
    SQLAlchemy model for a chunk of the book.
    Kitab ke ek chunk ke liye SQLAlchemy model.
    """
    __tablename__ = 'book_chunks'

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4())) # UUID as string
    text_content = Column(Text, nullable=False)
    chunk_metadata = Column(JSONB, nullable=False) # Store metadata as JSONB

    def __repr__(self):
        return f"<BookChunk(id={self.id}, source_file='{self.chunk_metadata.get('source_file')}')>"

class User(Base):
    """
    SQLAlchemy model for user authentication.
    User authentication ke liye SQLAlchemy model.
    """
    __tablename__ = 'users'

    id = Column(Integer, primary_key=True, index=True, autoincrement=True)
    first_name = Column(String(50), nullable=False)
    last_name = Column(String(50), nullable=False)
    email = Column(String(255), unique=True, index=True, nullable=False)
    hashed_password = Column(String(255), nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def __repr__(self):
        return f"<User(id={self.id}, email='{self.email}', name='{self.first_name} {self.last_name}')>"

# --- Database Engine Setup ---
engine = create_engine(DATABASE_URL)

# --- Session Setup ---
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

def get_db():
    """
    Dependency for FastAPI to get a DB session.
    FastAPI ke liye DB session hasil karne ki dependency.
    """
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def create_tables():
    """
    Creates all tables in the database.
    Database mein sabhi tables banata hai.
    """
    print("Creating database tables if they don't exist...")
    Base.metadata.create_all(bind=engine)
    print("Tables created.")

if __name__ == '__main__':
    # This will create the tables when you run this file directly
    # Jab aap is file ko seedhe chalayenge to yeh tables banayega
    create_tables()
