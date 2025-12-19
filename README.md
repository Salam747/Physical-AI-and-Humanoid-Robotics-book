# 🤖 Physical AI & Humanoid Robotics Book

> **An interactive learning platform powered by AI chatbot with RAG (Retrieval-Augmented Generation)**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![Node](https://img.shields.io/badge/Node-16+-green.svg)](https://nodejs.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.104+-009688.svg)](https://fastapi.tiangolo.com/)
[![React](https://img.shields.io/badge/React-18+-61DAFB.svg)](https://reactjs.org/)

---

## 🌟 Features

### 📚 **Educational Content**
- 4 comprehensive modules covering ROS 2, Digital Twins, RL, and VLA
- 20+ detailed chapters
- Interactive learning experience
- 100% free and open source

### 🤖 **AI-Powered Chatbot**
- **RAG (Retrieval-Augmented Generation)** for accurate answers
- **Gemini AI** integration for natural conversations
- **Context-aware** responses based on book content
- **Real-time** question answering
- Beautiful **glassmorphism** design

### 🔐 **Secure Authentication**
- JWT token-based authentication
- Secure password hashing (bcrypt)
- User registration and login
- Session management
- Protected chatbot access

### 🎨 **Premium UI/UX**
- Modern glassmorphism design
- Gradient backgrounds with floating orbs
- Smooth animations and transitions
- Fully responsive layout
- Premium loading states
- Success animations

---

## 🚀 Quick Start

### ⚡ **Option 1: Auto-Start (Easiest!)**

Just double-click:
```
START_HERE.bat
```

✅ Done! Both servers start automatically and browser opens!

### 🛠️ **Option 2: Manual Start**

#### **Terminal 1 - Backend:**
```bash
cd backend
uvicorn app:app --reload --port 8000
```

#### **Terminal 2 - Frontend:**
```bash
cd "Robotic book"
npm start -- --port 3002
```

#### **Open Browser:**
```
http://localhost:3002
```

---

## 🔐 Test Credentials

```
Email:    test@example.com
Password: Test@123
```

---

## 📖 Documentation

| File | Description |
|------|-------------|
| [QUICK_START.md](./QUICK_START.md) | ⚡ 3-step quick start guide |
| [SETUP_GUIDE.md](./SETUP_GUIDE.md) | 📖 Comprehensive setup instructions |
| [start-backend.bat](./start-backend.bat) | ⚙️ Backend launcher script |
| [start-frontend.bat](./start-frontend.bat) | 🎨 Frontend launcher script |
| [START_HERE.bat](./START_HERE.bat) | 🚀 Complete app launcher |

---

## 🏗️ Tech Stack

### **Backend:**
- **FastAPI** - Modern Python web framework
- **PostgreSQL** - User data & chat history
- **Qdrant** - Vector database for RAG
- **Google Gemini** - AI language model
- **JWT** - Authentication tokens
- **bcrypt** - Password hashing

### **Frontend:**
- **React** - UI library
- **TypeScript** - Type safety
- **Docusaurus** - Documentation framework
- **CSS Modules** - Scoped styling
- **Glassmorphism** - Premium UI design

### **RAG Pipeline:**
- **Embeddings** - Google Gemini text-embedding-004
- **Vector Store** - Qdrant cloud
- **Retrieval** - Semantic search
- **Generation** - Gemini 1.5 Pro

---

## 📁 Project Structure

```
Physical_AI_and_Humanoid_Robotics_book/
│
├── 📂 backend/                  # FastAPI Backend
│   ├── app.py                  # Main server
│   ├── auth/                   # Authentication
│   ├── chat/                   # Chat history
│   ├── db/                     # Database models
│   ├── rag/                    # RAG pipeline
│   │   ├── pipeline.py         # Main RAG logic
│   │   ├── retriever.py        # Qdrant search
│   │   └── generator.py        # Gemini response
│   └── .env                    # Environment vars
│
├── 📂 Robotic book/             # Docusaurus Frontend
│   ├── src/
│   │   ├── components/
│   │   │   ├── Auth/           # Login/Signup
│   │   │   │   ├── LoginForm.tsx
│   │   │   │   ├── SignUpForm.tsx
│   │   │   │   └── AuthModal.tsx
│   │   │   ├── Chatbot/        # AI Chatbot
│   │   │   │   ├── index.tsx
│   │   │   │   └── styles.module.css
│   │   │   └── HomepageFeatures/
│   │   ├── pages/
│   │   │   ├── index.tsx       # Homepage
│   │   │   └── login.tsx       # Login page
│   │   ├── context/
│   │   │   └── AuthContext.tsx # Auth state
│   │   └── utils/
│   │       └── auth.ts         # Auth helpers
│   └── docs/                   # Course content
│       ├── module1-ros2/
│       ├── module2-digital-twin/
│       ├── module3-ai-brain/
│       └── module4-vla/
│
├── 📜 START_HERE.bat           # Auto-launcher
├── 📜 start-backend.bat        # Backend script
├── 📜 start-frontend.bat       # Frontend script
├── 📖 QUICK_START.md           # Quick guide
├── 📖 SETUP_GUIDE.md           # Detailed guide
└── 📖 README.md                # This file
```

---

## 🎯 Key Features Explained

### 🔐 **Authentication System**
- User registration with validation
- Secure login with JWT tokens
- Password requirements (8+ chars, uppercase, number, special)
- Token expiration handling
- Protected routes

### 🤖 **RAG Chatbot**
1. **User asks question** → Frontend sends to backend
2. **Embedding creation** → Question converted to vector
3. **Semantic search** → Qdrant finds relevant book sections
4. **Context retrieval** → Top matching chunks retrieved
5. **AI generation** → Gemini generates answer with context
6. **Response** → Beautiful chat message with sources

### 🎨 **UI Components**
- **Glassmorphism Cards** - Frosted glass effect with blur
- **Gradient Backgrounds** - Purple → Violet → Pink
- **Floating Orbs** - Animated background elements
- **Smooth Transitions** - Cubic-bezier animations
- **Loading States** - Spinners and skeleton screens
- **Success Animations** - Checkmarks and celebrations

---

## 💬 Sample Chatbot Questions

Try asking:
1. **"What is ROS 2 and why use it?"**
2. **"Explain digital twins in robotics"**
3. **"How does reinforcement learning work for robots?"**
4. **"What are Vision-Language-Action models?"**
5. **"Tell me about NVIDIA Isaac Sim"**
6. **"How do I create custom ROS 2 messages?"**
7. **"What is PPO algorithm?"**
8. **"Explain sim-to-real transfer"**

---

## 🐛 Troubleshooting

### **Backend won't start?**
```bash
# Check port 8000
netstat -ano | findstr :8000

# Kill if busy
taskkill /PID <number> /F

# Restart
cd backend
uvicorn app:app --reload --port 8000
```

### **Frontend won't start?**
```bash
# Check port 3002
netstat -ano | findstr :3002

# Kill if busy
taskkill /PID <number> /F

# Restart
cd "Robotic book"
npm start -- --port 3002
```

### **Chatbot not appearing?**
1. ✅ Login first with credentials
2. 👀 Look for robot icon (bottom-right)
3. 🔧 Check backend health: http://127.0.0.1:8000/health
4. 📊 Check browser console (F12)

### **Can't login?**
1. Verify backend is running
2. Check database connection
3. Use correct password format (Test@123)
4. Try creating new user via signup

---

## 📞 API Endpoints

### **Authentication:**
- `POST /auth/signup` - Create new user
- `POST /auth/login` - Login user
- `GET /auth/verify` - Verify JWT token

### **Chat:**
- `POST /query` - Ask chatbot (requires auth)
- `GET /chat/history` - Get chat history

### **System:**
- `GET /health` - Server health check
- `POST /ingest` - Ingest book content

### **Interactive Docs:**
- http://127.0.0.1:8000/docs (Swagger UI)
- http://127.0.0.1:8000/redoc (ReDoc)

---

## 🌐 URLs

| Service | URL | Description |
|---------|-----|-------------|
| **Frontend** | http://localhost:3002 | Main website |
| **Backend** | http://127.0.0.1:8000 | API server |
| **API Docs** | http://127.0.0.1:8000/docs | Swagger UI |
| **Health** | http://127.0.0.1:8000/health | Health check |

---

## 🎓 What You'll Learn

### **Module 1: ROS 2 Fundamentals**
- ROS 2 architecture & concepts
- Creating custom messages
- Robot control basics
- Simulation with Gazebo

### **Module 2: Digital Twins**
- NVIDIA Isaac Sim
- Physics simulation
- Sensor integration
- URDF/SDF modeling

### **Module 3: Reinforcement Learning**
- PPO & SAC algorithms
- Reward shaping
- Curriculum learning
- Policy training

### **Module 4: Vision-Language-Action**
- LLM integration
- Vision models
- Action planning
- Natural language control

---

## 🛡️ Security Features

- ✅ JWT token authentication
- ✅ Bcrypt password hashing
- ✅ SQL injection protection
- ✅ CORS configuration
- ✅ Input validation
- ✅ Environment variables for secrets

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 👨‍💻 Author

**Physical AI & Robotics Team**

---

## 🌟 Show Your Support

Give a ⭐️ if this project helped you!

---

## 🙏 Acknowledgments

- **Google Gemini** for AI capabilities
- **FastAPI** for the amazing backend framework
- **Docusaurus** for the documentation platform
- **Qdrant** for vector database
- **React** community for UI components

---

**Happy Learning! 🤖✨**

**Made with ❤️ for Physical AI & Humanoid Robotics enthusiasts worldwide**
