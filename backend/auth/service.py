# Authentication service - Business logic
# Authentication ke liye business logic

from sqlalchemy.orm import Session
from db.models import User
from auth.utils import hash_password, verify_password, create_access_token
from auth.schemas import SignupRequest, LoginRequest, AuthResponse, UserResponse, VerifyTokenResponse
from fastapi import HTTPException


class AuthService:
    """
    Authentication service to handle user signup, login, and token verification.
    User signup, login, aur token verification handle karne ke liye service.
    """

    def __init__(self, db: Session):
        self.db = db

    def signup(self, data: SignupRequest) -> AuthResponse:
        """
        Register a new user.
        Naye user ko register karna.

        Args:
            data: SignupRequest with user details

        Returns:
            AuthResponse with token and user data

        Raises:
            HTTPException: If email already exists
        """
        # Check if user already exists
        existing_user = self.db.query(User).filter(User.email == data.email.lower()).first()
        if existing_user:
            raise HTTPException(
                status_code=400,
                detail="Email already registered. Please use a different email or login."
            )

        # Create new user
        new_user = User(
            first_name=data.first_name.strip(),
            last_name=data.last_name.strip(),
            email=data.email.lower(),
            hashed_password=hash_password(data.password)
        )

        self.db.add(new_user)
        self.db.commit()
        self.db.refresh(new_user)

        # Generate JWT token
        token = create_access_token({
            "sub": str(new_user.id),
            "email": new_user.email,
            "first_name": new_user.first_name,
            "last_name": new_user.last_name
        })

        # Return response
        return AuthResponse(
            success=True,
            message="User created successfully",
            token=token,
            user=UserResponse.model_validate(new_user)
        )

    def login(self, data: LoginRequest) -> AuthResponse:
        """
        Authenticate user and generate token.
        User ko authenticate karke token generate karna.

        Args:
            data: LoginRequest with email and password

        Returns:
            AuthResponse with token and user data

        Raises:
            HTTPException: If credentials are invalid
        """
        # Find user by email
        user = self.db.query(User).filter(User.email == data.email.lower()).first()

        # Verify user exists and password is correct
        if not user or not verify_password(data.password, user.hashed_password):
            raise HTTPException(
                status_code=401,
                detail="Invalid email or password. Please check your credentials and try again."
            )

        # Generate JWT token
        token = create_access_token({
            "sub": str(user.id),
            "email": user.email,
            "first_name": user.first_name,
            "last_name": user.last_name
        })

        # Return response
        return AuthResponse(
            success=True,
            message="Login successful",
            token=token,
            user=UserResponse.model_validate(user)
        )

    def verify_token_and_get_user(self, user_id: int) -> UserResponse:
        """
        Verify token and retrieve user data.
        Token verify karke user data nikalna.

        Args:
            user_id: User ID from JWT token

        Returns:
            UserResponse with user data

        Raises:
            HTTPException: If user not found
        """
        user = self.db.query(User).filter(User.id == user_id).first()

        if not user:
            raise HTTPException(
                status_code=404,
                detail="User not found. Token may be invalid or user has been deleted."
            )

        return UserResponse.model_validate(user)
