# Authentication System Implementation Plan

## Phase 1: Backend Foundation (Database + Models)

### Step 1.1: Update Database Models
**File**: `backend/db/models.py`

**Actions**:
1. Add `User` model with SQLAlchemy ORM:
   ```python
   class User(Base):
       __tablename__ = "users"
       id = Column(Integer, primary_key=True, index=True)
       first_name = Column(String(50), nullable=False)
       last_name = Column(String(50), nullable=False)
       email = Column(String(255), unique=True, index=True, nullable=False)
       hashed_password = Column(String(255), nullable=False)
       created_at = Column(DateTime, default=datetime.utcnow)
       updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
   ```
2. Create database migration script to add users table

**Files Created/Modified**:
- `backend/db/models.py` (modify)
- `backend/db/migrations/create_users_table.sql` (create)

---

### Step 1.2: Install Authentication Dependencies
**File**: `backend/requirements.txt`

**Actions**:
1. Add dependencies:
   ```
   python-jose[cryptography]==3.3.0
   passlib[bcrypt]==1.7.4
   python-multipart==0.0.6
   ```
2. Run `pip install -r requirements.txt`

**Files Modified**:
- `backend/requirements.txt`

---

### Step 1.3: Update Environment Variables
**File**: `backend/.env`

**Actions**:
1. Add JWT configuration:
   ```
   JWT_SECRET_KEY=your-256-bit-secret-key-here-must-be-at-least-32-characters-long
   JWT_ALGORITHM=HS256
   JWT_EXPIRY_HOURS=24
   ```

**Files Modified**:
- `backend/.env`

---

## Phase 2: Authentication Backend Logic

### Step 2.1: Create Authentication Utilities
**File**: `backend/auth/utils.py`

**Actions**:
1. Create password hashing functions:
   ```python
   from passlib.context import CryptContext

   pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

   def hash_password(password: str) -> str:
       return pwd_context.hash(password)

   def verify_password(plain_password: str, hashed_password: str) -> bool:
       return pwd_context.verify(plain_password, hashed_password)
   ```

2. Create JWT token functions:
   ```python
   from jose import JWTError, jwt
   from datetime import datetime, timedelta

   def create_access_token(data: dict) -> str:
       to_encode = data.copy()
       expire = datetime.utcnow() + timedelta(hours=24)
       to_encode.update({"exp": expire})
       return jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)

   def verify_token(token: str) -> dict:
       try:
           payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
           return payload
       except JWTError:
           return None
   ```

**Files Created**:
- `backend/auth/__init__.py`
- `backend/auth/utils.py`

---

### Step 2.2: Create Pydantic Schemas
**File**: `backend/auth/schemas.py`

**Actions**:
1. Create request/response models:
   ```python
   from pydantic import BaseModel, EmailStr, field_validator

   class SignupRequest(BaseModel):
       first_name: str
       last_name: str
       email: EmailStr
       password: str
       confirm_password: str

       @field_validator('first_name', 'last_name')
       def validate_name(cls, v):
           if len(v) < 2 or len(v) > 50:
               raise ValueError('Name must be between 2-50 characters')
           return v

       @field_validator('password')
       def validate_password(cls, v):
           if len(v) < 8:
               raise ValueError('Password must be at least 8 characters')
           # Add complexity checks
           return v

   class LoginRequest(BaseModel):
       email: EmailStr
       password: str

   class UserResponse(BaseModel):
       id: int
       first_name: str
       last_name: str
       email: str

   class AuthResponse(BaseModel):
       success: bool
       message: str
       token: str
       user: UserResponse
   ```

**Files Created**:
- `backend/auth/schemas.py`

---

### Step 2.3: Create Authentication Service
**File**: `backend/auth/service.py`

**Actions**:
1. Implement business logic:
   ```python
   from sqlalchemy.orm import Session
   from backend.db.models import User
   from backend.auth.utils import hash_password, verify_password, create_access_token

   class AuthService:
       def __init__(self, db: Session):
           self.db = db

       def signup(self, data: SignupRequest) -> AuthResponse:
           # Check if user exists
           existing_user = self.db.query(User).filter(User.email == data.email).first()
           if existing_user:
               raise ValueError("Email already registered")

           # Validate password confirmation
           if data.password != data.confirm_password:
               raise ValueError("Passwords do not match")

           # Create user
           user = User(
               first_name=data.first_name,
               last_name=data.last_name,
               email=data.email,
               hashed_password=hash_password(data.password)
           )
           self.db.add(user)
           self.db.commit()
           self.db.refresh(user)

           # Generate token
           token = create_access_token({
               "sub": str(user.id),
               "email": user.email,
               "first_name": user.first_name,
               "last_name": user.last_name
           })

           return AuthResponse(
               success=True,
               message="User created successfully",
               token=token,
               user=UserResponse.from_orm(user)
           )

       def login(self, data: LoginRequest) -> AuthResponse:
           # Find user
           user = self.db.query(User).filter(User.email == data.email).first()
           if not user or not verify_password(data.password, user.hashed_password):
               raise ValueError("Invalid credentials")

           # Generate token
           token = create_access_token({
               "sub": str(user.id),
               "email": user.email,
               "first_name": user.first_name,
               "last_name": user.last_name
           })

           return AuthResponse(
               success=True,
               message="Login successful",
               token=token,
               user=UserResponse.from_orm(user)
           )

       def verify_token(self, token: str) -> UserResponse:
           payload = verify_token(token)
           if not payload:
               raise ValueError("Invalid token")

           user = self.db.query(User).filter(User.id == int(payload["sub"])).first()
           if not user:
               raise ValueError("User not found")

           return UserResponse.from_orm(user)
   ```

**Files Created**:
- `backend/auth/service.py`

---

### Step 2.4: Create API Routes
**File**: `backend/auth/routes.py`

**Actions**:
1. Create FastAPI endpoints:
   ```python
   from fastapi import APIRouter, Depends, HTTPException, Header
   from sqlalchemy.orm import Session
   from backend.auth.service import AuthService
   from backend.auth.schemas import SignupRequest, LoginRequest, AuthResponse
   from backend.db.models import get_db

   router = APIRouter(prefix="/auth", tags=["Authentication"])

   @router.post("/signup", response_model=AuthResponse)
   async def signup(data: SignupRequest, db: Session = Depends(get_db)):
       try:
           service = AuthService(db)
           return service.signup(data)
       except ValueError as e:
           raise HTTPException(status_code=400, detail=str(e))

   @router.post("/login", response_model=AuthResponse)
   async def login(data: LoginRequest, db: Session = Depends(get_db)):
       try:
           service = AuthService(db)
           return service.login(data)
       except ValueError as e:
           raise HTTPException(status_code=401, detail=str(e))

   @router.get("/verify")
   async def verify_token(authorization: str = Header(None), db: Session = Depends(get_db)):
       if not authorization or not authorization.startswith("Bearer "):
           raise HTTPException(status_code=401, detail="Invalid authorization header")

       token = authorization.split(" ")[1]
       try:
           service = AuthService(db)
           user = service.verify_token(token)
           return {"success": True, "user": user}
       except ValueError as e:
           raise HTTPException(status_code=401, detail=str(e))
   ```

**Files Created**:
- `backend/auth/routes.py`

---

### Step 2.5: Update Main Application
**File**: `backend/app.py`

**Actions**:
1. Import and include auth router:
   ```python
   from backend.auth.routes import router as auth_router

   app.include_router(auth_router)
   ```
2. Update CORS origins if needed

**Files Modified**:
- `backend/app.py`

---

## Phase 3: Frontend Authentication UI

### Step 3.1: Install Frontend Dependencies
**File**: `Robotic book/package.json`

**Actions**:
1. Add dependencies:
   ```json
   {
     "dependencies": {
       "jwt-decode": "^4.0.0",
       "react-hook-form": "^7.49.0"
     }
   }
   ```
2. Run `npm install`

**Files Modified**:
- `Robotic book/package.json`

---

### Step 3.2: Create Authentication Context
**File**: `Robotic book/src/context/AuthContext.tsx`

**Actions**:
1. Create React Context for auth state:
   ```typescript
   import React, { createContext, useState, useEffect, useContext } from 'react';
   import { jwtDecode } from 'jwt-decode';

   interface User {
       id: number;
       first_name: string;
       last_name: string;
       email: string;
   }

   interface AuthContextType {
       user: User | null;
       token: string | null;
       login: (email: string, password: string) => Promise<void>;
       signup: (data: SignupData) => Promise<void>;
       logout: () => void;
       isAuthenticated: boolean;
       verifyToken: () => Promise<boolean>;
   }

   export const AuthContext = createContext<AuthContextType>(null);

   export const AuthProvider: React.FC = ({ children }) => {
       const [user, setUser] = useState<User | null>(null);
       const [token, setToken] = useState<string | null>(
           localStorage.getItem('auth_token')
       );

       // Implementation...
   };
   ```

**Files Created**:
- `Robotic book/src/context/AuthContext.tsx`

---

### Step 3.3: Create Authentication Utilities
**File**: `Robotic book/src/utils/auth.ts`

**Actions**:
1. Create helper functions:
   ```typescript
   export const API_URL = process.env.NODE_ENV === 'production'
       ? 'https://your-vercel-backend.vercel.app'
       : 'http://127.0.0.1:8000';

   export const setAuthToken = (token: string) => {
       localStorage.setItem('auth_token', token);
   };

   export const getAuthToken = (): string | null => {
       return localStorage.getItem('auth_token');
   };

   export const removeAuthToken = () => {
       localStorage.removeItem('auth_token');
   };

   export const isTokenExpired = (token: string): boolean => {
       try {
           const decoded: any = jwtDecode(token);
           return decoded.exp * 1000 < Date.now();
       } catch {
           return true;
       }
   };
   ```

**Files Created**:
- `Robotic book/src/utils/auth.ts`

---

### Step 3.4: Create Sign Up Form Component
**File**: `Robotic book/src/components/Auth/SignUpForm.tsx`

**Actions**:
1. Create form with validation:
   ```typescript
   import React, { useState } from 'react';
   import { useForm } from 'react-hook-form';
   import { useAuth } from '@site/src/context/AuthContext';

   interface SignUpFormData {
       first_name: string;
       last_name: string;
       email: string;
       password: string;
       confirm_password: string;
   }

   export default function SignUpForm() {
       const { register, handleSubmit, formState: { errors }, watch } = useForm();
       const { signup } = useAuth();
       const [error, setError] = useState('');

       const onSubmit = async (data: SignUpFormData) => {
           try {
               await signup(data);
               // Redirect handled in AuthContext
           } catch (err) {
               setError(err.message);
           }
       };

       return (
           <form onSubmit={handleSubmit(onSubmit)}>
               {/* Form fields with validation */}
           </form>
       );
   }
   ```

**Files Created**:
- `Robotic book/src/components/Auth/SignUpForm.tsx`
- `Robotic book/src/components/Auth/styles.module.css`

---

### Step 3.5: Create Log In Form Component
**File**: `Robotic book/src/components/Auth/LoginForm.tsx`

**Actions**:
1. Create login form:
   ```typescript
   import React, { useState } from 'react';
   import { useForm } from 'react-hook-form';
   import { useAuth } from '@site/src/context/AuthContext';

   interface LoginFormData {
       email: string;
       password: string;
   }

   export default function LoginForm() {
       const { register, handleSubmit, formState: { errors } } = useForm();
       const { login } = useAuth();
       const [error, setError] = useState('');

       const onSubmit = async (data: LoginFormData) => {
           try {
               await login(data.email, data.password);
               // Redirect handled in AuthContext
           } catch (err) {
               setError(err.message);
           }
       };

       return (
           <form onSubmit={handleSubmit(onSubmit)}>
               {/* Form fields */}
           </form>
       );
   }
   ```

**Files Created**:
- `Robotic book/src/components/Auth/LoginForm.tsx`

---

### Step 3.6: Create Login Page with Tabs
**File**: `Robotic book/src/pages/login.tsx`

**Actions**:
1. Create two-tab interface:
   ```typescript
   import React, { useState } from 'react';
   import Layout from '@theme/Layout';
   import SignUpForm from '@site/src/components/Auth/SignUpForm';
   import LoginForm from '@site/src/components/Auth/LoginForm';

   export default function LoginPage() {
       const [activeTab, setActiveTab] = useState<'login' | 'signup'>('login');

       return (
           <Layout title="Login">
               <div className="container">
                   <div className="auth-container">
                       <div className="tabs">
                           <button onClick={() => setActiveTab('login')}>
                               Log In
                           </button>
                           <button onClick={() => setActiveTab('signup')}>
                               Sign Up
                           </button>
                       </div>
                       {activeTab === 'login' ? <LoginForm /> : <SignUpForm />}
                   </div>
               </div>
           </Layout>
       );
   }
   ```

**Files Created**:
- `Robotic book/src/pages/login.tsx`
- `Robotic book/src/pages/login.module.css`

---

### Step 3.7: Create Protected Route Component
**File**: `Robotic book/src/components/Auth/ProtectedRoute.tsx`

**Actions**:
1. Create HOC for route protection:
   ```typescript
   import React, { useEffect, useState } from 'react';
   import { useAuth } from '@site/src/context/AuthContext';
   import { useHistory } from '@docusaurus/router';

   export default function ProtectedRoute({ children }) {
       const { isAuthenticated, verifyToken } = useAuth();
       const history = useHistory();
       const [loading, setLoading] = useState(true);

       useEffect(() => {
           const checkAuth = async () => {
               const valid = await verifyToken();
               if (!valid) {
                   history.push('/login');
               }
               setLoading(false);
           };
           checkAuth();
       }, []);

       if (loading) return <div>Loading...</div>;
       return isAuthenticated ? children : null;
   }
   ```

**Files Created**:
- `Robotic book/src/components/Auth/ProtectedRoute.tsx`

---

### Step 3.8: Create Protected Chatbot Page
**File**: `Robotic book/src/pages/chatbot.tsx`

**Actions**:
1. Move chatbot component to dedicated page:
   ```typescript
   import React from 'react';
   import Layout from '@theme/Layout';
   import ProtectedRoute from '@site/src/components/Auth/ProtectedRoute';
   import Chatbot from '@site/src/components/Chatbot';
   import { useAuth } from '@site/src/context/AuthContext';

   export default function ChatbotPage() {
       const { user, logout } = useAuth();

       return (
           <Layout title="AI Chatbot">
               <ProtectedRoute>
                   <div className="chatbot-page">
                       <div className="chatbot-header">
                           <h1>AI Chatbot</h1>
                           <div className="user-info">
                               <span>Welcome, {user?.first_name}!</span>
                               <button onClick={logout}>Logout</button>
                           </div>
                       </div>
                       <Chatbot />
                   </div>
               </ProtectedRoute>
           </Layout>
       );
   }
   ```

**Files Created**:
- `Robotic book/src/pages/chatbot.tsx`
- `Robotic book/src/pages/chatbot.module.css`

---

## Phase 4: Integration & Testing

### Step 4.1: Update Chatbot Component
**File**: `Robotic book/src/components/Chatbot/index.tsx`

**Actions**:
1. Add JWT token to API requests:
   ```typescript
   const { token } = useAuth();

   const response = await fetch(`${API_URL}/query`, {
       method: 'POST',
       headers: {
           'Content-Type': 'application/json',
           'Authorization': `Bearer ${token}`
       },
       body: JSON.stringify({ question: inputValue })
   });
   ```

**Files Modified**:
- `Robotic book/src/components/Chatbot/index.tsx`

---

### Step 4.2: Update Navbar
**File**: `Robotic book/docusaurus.config.ts`

**Actions**:
1. Update navbar links:
   ```typescript
   navbar: {
       items: [
           {
               type: 'doc',
               docId: 'intro',
               position: 'left',
               label: 'Read Book',
           },
           {
               to: '/chatbot',
               label: 'AI Chatbot',
               position: 'left',
           },
           // Auth-specific items handled in custom navbar component
       ],
   }
   ```

**Files Modified**:
- `Robotic book/docusaurus.config.ts`

---

### Step 4.3: Wrap App with Auth Provider
**File**: `Robotic book/src/theme/Root.tsx`

**Actions**:
1. Create Root wrapper:
   ```typescript
   import React from 'react';
   import { AuthProvider } from '@site/src/context/AuthContext';

   export default function Root({ children }) {
       return (
           <AuthProvider>
               {children}
           </AuthProvider>
       );
   }
   ```

**Files Created**:
- `Robotic book/src/theme/Root.tsx`

---

### Step 4.4: Run Database Migration
**Command**: Run SQL migration

**Actions**:
1. Connect to Neon database
2. Execute `backend/db/migrations/create_users_table.sql`
3. Verify table creation:
   ```sql
   SELECT * FROM users LIMIT 1;
   ```

---

### Step 4.5: Test Backend Endpoints
**Command**: Test API with curl/Postman

**Actions**:
1. Test signup endpoint:
   ```bash
   curl -X POST http://localhost:8000/auth/signup \
     -H "Content-Type: application/json" \
     -d '{
       "first_name": "John",
       "last_name": "Doe",
       "email": "john@test.com",
       "password": "Test123!",
       "confirm_password": "Test123!"
     }'
   ```

2. Test login endpoint:
   ```bash
   curl -X POST http://localhost:8000/auth/login \
     -H "Content-Type: application/json" \
     -d '{
       "email": "john@test.com",
       "password": "Test123!"
     }'
   ```

3. Test verify endpoint:
   ```bash
   curl -X GET http://localhost:8000/auth/verify \
     -H "Authorization: Bearer <token>"
   ```

---

### Step 4.6: Test Frontend Flow
**Command**: Manual testing

**Actions**:
1. Start frontend: `npm start`
2. Test signup flow:
   - Navigate to `/login`
   - Fill signup form
   - Verify redirect to `/chatbot`
3. Test login flow:
   - Logout
   - Login with created credentials
   - Verify redirect to `/chatbot`
4. Test protection:
   - Clear localStorage
   - Try accessing `/chatbot`
   - Verify redirect to `/login`
5. Test logout:
   - Click logout button
   - Verify token cleared
   - Verify redirect to `/login`

---

## Phase 5: Deployment

### Step 5.1: Update Environment Variables in Vercel
**Platform**: Vercel Dashboard

**Actions**:
1. Add environment variables:
   - `JWT_SECRET_KEY`
   - `JWT_ALGORITHM`
   - `JWT_EXPIRY_HOURS`
   - `DATABASE_URL` (if not already set)

---

### Step 5.2: Deploy Backend
**Command**: Git push to trigger Vercel deployment

**Actions**:
1. Commit all backend changes
2. Push to main branch
3. Verify Vercel deployment
4. Test production endpoints

---

### Step 5.3: Deploy Frontend
**Command**: `npm run deploy`

**Actions**:
1. Update `API_URL` in `auth.ts` to production URL
2. Build and deploy to GitHub Pages
3. Test production authentication flow

---

### Step 5.4: Final Verification
**Actions**:
1. Test signup on production
2. Test login on production
3. Test chatbot access on production
4. Test logout on production
5. Verify book pages remain public
6. Check CORS settings
7. Monitor error logs

---

## Rollback Plan

If issues occur:
1. Revert backend changes: `git revert <commit-hash>`
2. Revert frontend changes: `git revert <commit-hash>`
3. Remove auth router from `app.py`
4. Redeploy previous stable version
5. Book content remains accessible (no impact)

---

## Success Metrics

- [ ] Users can sign up successfully
- [ ] Users can log in successfully
- [ ] JWT tokens are generated and verified
- [ ] Chatbot page is protected
- [ ] Logout works correctly
- [ ] Book pages remain public
- [ ] No breaking changes to existing features
- [ ] Production deployment successful
- [ ] Error handling works as expected
- [ ] CORS configured correctly

---

## Estimated Implementation Time
- Phase 1: 1-2 hours
- Phase 2: 3-4 hours
- Phase 3: 4-5 hours
- Phase 4: 2-3 hours
- Phase 5: 1-2 hours

**Total**: 11-16 hours of development time
