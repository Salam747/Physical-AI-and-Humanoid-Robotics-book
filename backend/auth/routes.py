# FastAPI routes for authentication
# Authentication ke liye FastAPI routes

from fastapi import APIRouter, Depends, HTTPException, Header
from sqlalchemy.orm import Session
from db.models import get_db
from auth.service import AuthService
from auth.schemas import (
    SignupRequest,
    LoginRequest,
    AuthResponse,
    ErrorResponse,
    VerifyTokenResponse
)
from auth.utils import verify_token


router = APIRouter(prefix="/auth", tags=["Authentication"])


@router.post("/signup", response_model=AuthResponse, responses={400: {"model": ErrorResponse}})
async def signup(data: SignupRequest, db: Session = Depends(get_db)):
    """
    Register a new user.
    Naye user ko register karna.

    - **first_name**: User's first name (2-50 characters)
    - **last_name**: User's last name (2-50 characters)
    - **email**: Valid email address
    - **password**: Strong password (min 8 chars, uppercase, lowercase, number, special char)
    - **confirm_password**: Must match password

    Returns JWT token and user data on success.
    """
    try:
        service = AuthService(db)
        return service.signup(data)
    except HTTPException:
        raise
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        print(f"Signup error: {e}")
        raise HTTPException(status_code=500, detail="An error occurred during signup. Please try again.")


@router.post("/login", response_model=AuthResponse, responses={401: {"model": ErrorResponse}})
async def login(data: LoginRequest, db: Session = Depends(get_db)):
    """
    Authenticate user and generate JWT token.
    User ko authenticate karke JWT token generate karna.

    - **email**: User's email address
    - **password**: User's password

    Returns JWT token and user data on success.
    """
    try:
        service = AuthService(db)
        return service.login(data)
    except HTTPException:
        raise
    except Exception as e:
        print(f"Login error: {e}")
        raise HTTPException(status_code=500, detail="An error occurred during login. Please try again.")


@router.get("/verify", response_model=VerifyTokenResponse, responses={401: {"model": ErrorResponse}})
async def verify_token_endpoint(authorization: str = Header(None), db: Session = Depends(get_db)):
    """
    Verify JWT token and return user data.
    JWT token verify karke user data return karna.

    Requires Authorization header with format: "Bearer <token>"

    Returns user data if token is valid.
    """
    # Check if Authorization header exists
    if not authorization:
        raise HTTPException(
            status_code=401,
            detail="Authorization header missing. Please provide a valid JWT token."
        )

    # Check if Authorization header has correct format
    if not authorization.startswith("Bearer "):
        raise HTTPException(
            status_code=401,
            detail="Invalid authorization header format. Use: Bearer <token>"
        )

    # Extract token
    token = authorization.split(" ")[1]

    # Verify token
    payload = verify_token(token)
    if not payload:
        raise HTTPException(
            status_code=401,
            detail="Invalid or expired token. Please login again."
        )

    # Get user ID from token
    user_id = payload.get("sub")
    if not user_id:
        raise HTTPException(
            status_code=401,
            detail="Invalid token payload. Please login again."
        )

    # Get user from database
    try:
        service = AuthService(db)
        user = service.verify_token_and_get_user(int(user_id))
        return VerifyTokenResponse(success=True, user=user)
    except HTTPException:
        raise
    except Exception as e:
        print(f"Token verification error: {e}")
        raise HTTPException(
            status_code=500,
            detail="An error occurred during token verification. Please try again."
        )


@router.post("/logout", response_model=dict)
async def logout():
    """
    Logout endpoint (client-side token removal).
    Logout endpoint (client side pe token remove karna).

    Since JWT tokens are stateless, logout is handled client-side by removing the token.
    This endpoint exists for consistency and future enhancements (e.g., token blacklisting).

    Returns success message.
    """
    return {
        "success": True,
        "message": "Logout successful. Token should be removed from client storage."
    }
