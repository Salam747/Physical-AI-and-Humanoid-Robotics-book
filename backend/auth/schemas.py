# Pydantic schemas for authentication API
# Authentication API ke liye Pydantic schemas

from pydantic import BaseModel, EmailStr, field_validator, ConfigDict
import re


class SignupRequest(BaseModel):
    """Schema for user signup request"""
    first_name: str
    last_name: str
    email: EmailStr
    password: str
    confirm_password: str

    @field_validator('first_name', 'last_name')
    @classmethod
    def validate_name(cls, v: str) -> str:
        """Validate name fields"""
        v = v.strip()
        if len(v) < 2:
            raise ValueError('Name must be at least 2 characters long')
        if len(v) > 50:
            raise ValueError('Name must be at most 50 characters long')
        if not re.match(r'^[a-zA-Z\s\-\']+$', v):
            raise ValueError('Name can only contain letters, spaces, hyphens, and apostrophes')
        return v

    @field_validator('password')
    @classmethod
    def validate_password(cls, v: str) -> str:
        """Validate password complexity"""
        if len(v) < 6:
            raise ValueError('Password must be at least 6 characters long')
        if len(v) > 14:
            raise ValueError('Password must be at most 14 characters long')
        if not re.search(r'[A-Z]', v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not re.search(r'[a-z]', v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not re.search(r'[0-9]', v):
            raise ValueError('Password must contain at least one number')
        # Special character requirement removed
        return v

    def model_post_init(self, __context) -> None:
        """Validate password confirmation matches"""
        if self.password != self.confirm_password:
            raise ValueError('Passwords do not match')


class LoginRequest(BaseModel):
    """Schema for user login request"""
    email: EmailStr
    password: str


class UserResponse(BaseModel):
    """Schema for user data in responses"""
    model_config = ConfigDict(from_attributes=True)

    id: int
    first_name: str
    last_name: str
    email: str


class AuthResponse(BaseModel):
    """Schema for authentication responses (signup/login)"""
    success: bool
    message: str
    token: str
    user: UserResponse


class ErrorResponse(BaseModel):
    """Schema for error responses"""
    success: bool
    message: str


class VerifyTokenResponse(BaseModel):
    """Schema for token verification response"""
    success: bool
    user: UserResponse
