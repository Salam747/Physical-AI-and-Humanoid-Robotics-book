# Authentication System Specification

## Overview
Professional authentication system for Physical AI and Humanoid Robotics Book with JWT-based security. Book content remains completely free and public - only the AI Chatbot requires authentication.

## Core Requirements

### 1. Access Control
- **Public Access**: All book modules (ROS 2, Digital Twin, AI, VLA) remain completely free
- **Protected Access**: Only `/chatbot` page requires authentication
- **Entry Point**: `/login` page serves as authentication gateway

### 2. Authentication Flow

#### Sign Up Flow
1. User visits `/login` page
2. Selects "Sign Up" tab
3. Fills form:
   - First Name (required, 2-50 chars)
   - Last Name (required, 2-50 chars)
   - Email (required, valid email format)
   - Password (required, min 8 chars, must contain: uppercase, lowercase, number, special char)
   - Confirm Password (must match password)
4. On submission:
   - Validate all fields
   - Check if email already exists
   - Hash password with bcrypt
   - Save to Neon users table: `first_name`, `last_name`, `email`, `hashed_password`
   - Generate JWT token (24h expiry)
   - Save token in localStorage
   - Redirect to `/chatbot`

#### Log In Flow
1. User visits `/login` page
2. Selects "Log In" tab
3. Fills form:
   - Email
   - Password
4. On submission:
   - Find user by email
   - Verify password with bcrypt
   - Generate JWT token (24h expiry)
   - Save token in localStorage
   - Redirect to `/chatbot`

#### Chatbot Protection
1. User tries to access `/chatbot`
2. Frontend checks localStorage for JWT token
3. If token missing → redirect to `/login`
4. If token exists → verify with backend `/auth/verify` endpoint
5. If token invalid/expired → redirect to `/login`
6. If token valid → allow access to chatbot

#### Logout Flow
1. User clicks "Logout" button on chatbot page
2. Clear JWT token from localStorage
3. Redirect to `/login` page

### 3. Navbar Updates
- **Before Login**:
  - "Read Book" → always visible (free access)
  - "AI Chatbot" → redirects to `/login` if not authenticated
- **After Login**:
  - "Read Book" → always visible
  - "AI Chatbot" → accessible
  - "Logout" → clears session

## Technical Architecture

### Backend Components

#### 1. Database Schema (Neon PostgreSQL)

```sql
CREATE TABLE users (
    id SERIAL PRIMARY KEY,
    first_name VARCHAR(50) NOT NULL,
    last_name VARCHAR(50) NOT NULL,
    email VARCHAR(255) UNIQUE NOT NULL,
    hashed_password VARCHAR(255) NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
```

#### 2. API Endpoints

**Base URL**: `/auth`

| Endpoint | Method | Purpose | Auth Required |
|----------|--------|---------|---------------|
| `/auth/signup` | POST | Register new user | No |
| `/auth/login` | POST | Authenticate user | No |
| `/auth/verify` | GET | Verify JWT token | Yes (Bearer token) |
| `/auth/logout` | POST | Invalidate token (optional) | Yes |

**Request/Response Models**:

```python
# Signup Request
{
    "first_name": "John",
    "last_name": "Doe",
    "email": "john@example.com",
    "password": "SecurePass123!",
    "confirm_password": "SecurePass123!"
}

# Signup Response (Success)
{
    "success": true,
    "message": "User created successfully",
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "user": {
        "id": 1,
        "first_name": "John",
        "last_name": "Doe",
        "email": "john@example.com"
    }
}

# Login Request
{
    "email": "john@example.com",
    "password": "SecurePass123!"
}

# Login Response (Success)
{
    "success": true,
    "message": "Login successful",
    "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
    "user": {
        "id": 1,
        "first_name": "John",
        "last_name": "Doe",
        "email": "john@example.com"
    }
}

# Verify Token Response
{
    "success": true,
    "user": {
        "id": 1,
        "email": "john@example.com",
        "first_name": "John",
        "last_name": "Doe"
    }
}

# Error Response
{
    "success": false,
    "message": "Invalid credentials"
}
```

#### 3. JWT Token Structure

```json
{
    "sub": "user_id",
    "email": "john@example.com",
    "first_name": "John",
    "last_name": "Doe",
    "exp": 1735689600,
    "iat": 1735603200
}
```

**Token Configuration**:
- Algorithm: HS256
- Expiry: 24 hours
- Secret: Stored in `.env` as `JWT_SECRET_KEY`

### Frontend Components

#### 1. Pages

**`/login` Page**:
- Path: `Robotic book/src/pages/login.tsx`
- Two-tab interface (Sign Up / Log In)
- Form validation with real-time feedback
- Error message display
- Professional styling matching Docusaurus theme

**`/chatbot` Page**:
- Path: `Robotic book/src/pages/chatbot.tsx`
- Protected route with token verification
- Logout button in header
- Existing chatbot component integration

#### 2. Authentication Context

**`AuthContext.tsx`**:
```typescript
interface AuthContextType {
    user: User | null;
    token: string | null;
    login: (email: string, password: string) => Promise<void>;
    signup: (data: SignupData) => Promise<void>;
    logout: () => void;
    isAuthenticated: boolean;
    verifyToken: () => Promise<boolean>;
}
```

#### 3. Protected Route Component

```typescript
<ProtectedRoute>
    <ChatbotPage />
</ProtectedRoute>
```

### Security Measures

1. **Password Security**:
   - Minimum 8 characters
   - Must contain uppercase, lowercase, number, special character
   - Hashed with bcrypt (salt rounds: 12)
   - Never stored in plain text

2. **Token Security**:
   - Stored in localStorage (with httpOnly cookie as future enhancement)
   - 24-hour expiry
   - Verified on every protected route access
   - Invalidated on logout

3. **API Security**:
   - CORS restricted to known origins
   - Rate limiting on auth endpoints (10 requests/minute per IP)
   - Input validation and sanitization
   - SQL injection prevention via SQLAlchemy ORM

4. **Error Handling**:
   - Generic error messages (avoid leaking user existence)
   - Detailed logging on backend
   - User-friendly messages on frontend

## File Structure

```
backend/
├── auth/
│   ├── __init__.py
│   ├── models.py              # User model
│   ├── schemas.py             # Pydantic models
│   ├── service.py             # Business logic
│   ├── utils.py               # JWT, hashing utilities
│   └── routes.py              # FastAPI endpoints
├── middleware/
│   └── auth_middleware.py     # Token verification
├── requirements.txt           # Add: python-jose, passlib, bcrypt
└── .env                       # Add: JWT_SECRET_KEY

Robotic book/src/
├── pages/
│   ├── login.tsx              # Authentication page
│   └── chatbot.tsx            # Protected chatbot page
├── components/
│   ├── Auth/
│   │   ├── SignUpForm.tsx
│   │   ├── LoginForm.tsx
│   │   └── ProtectedRoute.tsx
│   └── Chatbot/               # Existing component
├── context/
│   └── AuthContext.tsx        # Auth state management
└── utils/
    └── auth.ts                # Token helpers
```

## Dependencies to Add

### Backend
```txt
python-jose[cryptography]==3.3.0  # JWT handling
passlib[bcrypt]==1.7.4            # Password hashing
python-multipart==0.0.6           # Form data parsing
```

### Frontend
```json
{
  "dependencies": {
    "jwt-decode": "^4.0.0",      // JWT decoding
    "react-hook-form": "^7.49.0" // Form management
  }
}
```

## Environment Variables

Add to `backend/.env`:
```env
JWT_SECRET_KEY=your-super-secret-key-here-min-32-chars
JWT_ALGORITHM=HS256
JWT_EXPIRY_HOURS=24
```

## CORS Update

Update `backend/app.py`:
```python
origins = [
    "https://salam747.github.io",
    "http://localhost:3000",
    "http://localhost",
]
```

## Testing Checklist

### Unit Tests
- [ ] Password hashing/verification
- [ ] JWT token generation/validation
- [ ] User creation with duplicate email
- [ ] Login with invalid credentials

### Integration Tests
- [ ] Complete signup flow
- [ ] Complete login flow
- [ ] Token verification endpoint
- [ ] Protected chatbot access

### E2E Tests
- [ ] User can sign up and access chatbot
- [ ] User can log in and access chatbot
- [ ] Unauthenticated user redirected to login
- [ ] Logout clears token and redirects
- [ ] Book pages remain accessible without login

## Success Criteria

1. ✓ Book content remains 100% free and accessible
2. ✓ Chatbot requires valid authentication
3. ✓ Professional login/signup UI with validation
4. ✓ Secure password storage (bcrypt)
5. ✓ JWT tokens with 24h expiry
6. ✓ Smooth redirects and UX flow
7. ✓ Logout functionality works correctly
8. ✓ Navbar updates based on auth state
9. ✓ No breaking changes to existing features
10. ✓ Production-ready error handling

## Future Enhancements
- Password reset via email
- Email verification
- OAuth integration (Google, GitHub)
- Remember me functionality
- Session management dashboard
- Two-factor authentication (2FA)
