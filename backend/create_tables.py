"""
Script to create database tables
Database tables banane ke liye script
"""

from db.models import create_tables

if __name__ == "__main__":
    print("Creating database tables...")
    create_tables()
    print("✓ Database tables created successfully!")
    print("\nTables created:")
    print("  - users")
    print("  - book_chunks")
    print("  - chat_messages")
