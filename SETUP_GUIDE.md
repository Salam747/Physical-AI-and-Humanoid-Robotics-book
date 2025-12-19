# 🚀 Setup Guide - Physical AI & Robotics Book

## 📋 Quick Start Guide for Zed Editor

### 🎯 Prerequisites
- Python 3.8+
- Node.js 16+
- PostgreSQL database
- Qdrant vector database (cloud or local)

---

## 🔧 Step 1: Backend Setup

### Open Terminal 1 (Backend)

```bash
# Navigate to backend directory
cd backend

# Activate virtual environment (if you have one)
# source venv/bin/activate  # On Linux/Mac
# venv\Scripts\activate     # On Windows

# Install dependencies (if not already installed)
pip install -r requirements.txt

# Create .env file with your credentials
# Make sure you have:
# - GOOGLE_API_KEY
# - DATABASE_URL
# - QDRANT_URL
# - QDRANT_API_KEY
# - JWT_SECRET_KEY

# Start the backend server
uvicorn app:app --reload --port 8000
```

**Expected Output:**
```
INFO:     Uvicorn running on http://127.0.0.1:8000
INFO:     Application startup complete.
```

**Test Backend:**
- Open browser: http://127.0.0.1:8000/health
- Should show: `{"status":"ok","message":"Server is healthy"}`

---

## 🎨 Step 2: Frontend Setup

### Open Terminal 2 (Frontend)

```bash
# Navigate to frontend directory
cd "Robotic book"

# Install dependencies (if not already installed)
npm install

# Start the development server
npm start -- --port 3002
```

**Expected Output:**
```
[SUCCESS] Docusaurus website is running at: http://localhost:3002/
```

---

## 🧪 Step 3: Test the Application

### 1️⃣ Open Application
- URL: **http://localhost:3002**
- You should see the beautiful homepage with gradient background

### 2️⃣ Test Login
- Click on **Login** button in navbar
- Or go to: **http://localhost:3002/login**

### 3️⃣ Use Test Credentials
```
Email:    test@example.com
Password: Test@123
```

### 4️⃣ Success!
- After login, you'll see a success animation ✓
- Automatically redirects to homepage
- Look for **robot icon 🤖** in bottom-right corner
- Click it to open the chatbot!

---

## 💬 Step 4: Use the Chatbot

### Sample Questions to Ask:
1. "What is ROS 2?"
2. "Tell me about digital twins in robotics"
3. "Explain reinforcement learning for robots"
4. "How do Vision-Language-Action models work?"
5. "What is NVIDIA Isaac Sim?"

---

## 🎨 Visual Features

### Homepage Design:
- ✨ Premium gradient background (purple → violet → pink)
- 🎴 Glassmorphism cards
- 🌈 Animated floating orbs
- 💫 Smooth hover effects
- 📱 Fully responsive

### Login Page Design:
- 🎭 Glassmorphism auth card
- 🔄 Smooth tab transitions
- 🌟 Glowing title with robot emoji
- ✓ Success animation with checkmark
- 🎨 Gradient backgrounds

### Chatbot Design:
- 🤖 Premium robot icon
- 💎 Glassmorphism window
- 💬 Smooth message animations
- ⚡ Real-time responses
- 🎯 Context-aware answers

---

## 🔐 Creating New Users

### Option 1: Via UI
1. Go to login page
2. Click "Sign up here" button
3. Fill the form with:
   - First Name
   - Last Name
   - Email
   - Password (must have: uppercase, lowercase, number, special char)
   - Confirm Password

### Option 2: Via API (cURL)
```bash
curl -X POST http://127.0.0.1:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "first_name": "John",
    "last_name": "Doe",
    "email": "john@example.com",
    "password": "SecurePass@123",
    "confirm_password": "SecurePass@123"
  }'
```

---

## 🐛 Troubleshooting

### Backend Not Starting?
```bash
# Check if port 8000 is already in use
netstat -ano | findstr :8000

# Kill the process if needed
taskkill /PID <PID_NUMBER> /F

# Restart backend
cd backend
uvicorn app:app --reload --port 8000
```

### Frontend Not Starting?
```bash
# Check if port 3002 is in use
netstat -ano | findstr :3002

# Kill the process if needed
taskkill /PID <PID_NUMBER> /F

# Restart frontend
cd "Robotic book"
npm start -- --port 3002
```

### Chatbot Not Appearing?
1. Make sure you're logged in
2. Check browser console for errors (F12)
3. Verify backend is running (http://127.0.0.1:8000/health)
4. Look for robot icon in **bottom-right corner**

### Login Not Working?
1. Verify backend is running
2. Check database connection in backend logs
3. Use correct password format (Test@123)
4. Try creating a new user

### Database Connection Error?
```bash
# Check .env file in backend directory
# Verify DATABASE_URL is correct
# Example: postgresql://user:password@localhost:5432/dbname
```

---

## 📁 Project Structure

```
Physical_AI_and_Humanoid_Robotics_book/
├── backend/                    # FastAPI backend
│   ├── app.py                 # Main server file
│   ├── auth/                  # Authentication routes
│   ├── chat/                  # Chat history routes
│   ├── db/                    # Database models
│   ├── rag/                   # RAG pipeline
│   └── .env                   # Environment variables
│
├── Robotic book/              # Docusaurus frontend
│   ├── src/
│   │   ├── components/
│   │   │   ├── Auth/         # Login/Signup components
│   │   │   └── Chatbot/      # Chatbot component
│   │   ├── pages/
│   │   │   ├── index.tsx     # Homepage
│   │   │   └── login.tsx     # Login page
│   │   └── context/
│   │       └── AuthContext.tsx
│   └── docusaurus.config.ts
│
└── SETUP_GUIDE.md             # This file!
```

---

## 🌟 Key Features

### Authentication:
- ✅ JWT token-based authentication
- ✅ Secure password hashing (bcrypt)
- ✅ Password validation (8+ chars, uppercase, number, special)
- ✅ Session management
- ✅ Token expiration handling

### Chatbot (RAG):
- ✅ Retrieval-Augmented Generation
- ✅ Gemini AI integration
- ✅ Qdrant vector database
- ✅ Context-aware responses
- ✅ Book content search
- ✅ Source citations

### UI/UX:
- ✅ Glassmorphism design
- ✅ Gradient backgrounds
- ✅ Smooth animations
- ✅ Responsive layout
- ✅ Loading states
- ✅ Error handling
- ✅ Success feedback

---

## 📞 API Endpoints

### Authentication:
- `POST /auth/signup` - Create new user
- `POST /auth/login` - Login user
- `GET /auth/verify` - Verify JWT token

### Chat:
- `POST /query` - Send message to chatbot (requires auth)
- `GET /health` - Check server status

### Admin:
- `POST /ingest` - Ingest book content to vector DB

---

## 🎓 Learning Resources

### Technologies Used:
- **Backend:** FastAPI, Python, PostgreSQL
- **Vector DB:** Qdrant
- **AI:** Google Gemini
- **Frontend:** React, TypeScript, Docusaurus
- **Auth:** JWT, bcrypt
- **Styling:** CSS Modules, Glassmorphism

---

## 🤝 Support

### Need Help?
1. Check troubleshooting section above
2. Review backend logs in terminal
3. Check browser console (F12)
4. Verify all environment variables are set

---

## ✨ Enjoy Your AI-Powered Robotics Learning Platform! 🤖

**Made with ❤️ for Physical AI & Humanoid Robotics enthusiasts**
