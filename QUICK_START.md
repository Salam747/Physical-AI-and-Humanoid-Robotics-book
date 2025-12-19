# ⚡ Quick Start - 3 Simple Steps

## 🎯 Option 1: Automatic Start (EASIEST!)

### Just Double-Click:
```
START_HERE.bat
```

✅ **Done!** Both servers will start automatically and browser will open!

---

## 🎯 Option 2: Manual Start (Zed Editor)

### Step 1: Open Zed Editor
```bash
# Open project in Zed
zed .
```

### Step 2: Open Terminal (in Zed)
Press: **Ctrl + `** (or View → Terminal)

### Step 3: Start Backend
```bash
# Terminal 1
cd backend
uvicorn app:app --reload --port 8000
```

### Step 4: Start Frontend (New Terminal)
```bash
# Terminal 2 (Ctrl + Shift + `)
cd "Robotic book"
npm start -- --port 3002
```

### Step 5: Open Browser
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

## 💬 Try These Questions in Chatbot:

1. **"What is ROS 2?"**
2. **"Tell me about digital twins"**
3. **"Explain reinforcement learning"**
4. **"How do VLA models work?"**

---

## 🎨 What You'll See:

### Homepage (localhost:3002)
- 🌈 Beautiful gradient background
- 🎴 Glassmorphism cards
- 💫 Smooth animations
- 🤖 Robot chatbot icon (bottom-right)

### Login Page (localhost:3002/login)
- 🎭 Premium glass design
- ✨ Glowing title
- 🔄 Smooth tab switching
- ✓ Success animation

### Chatbot
- 💬 Click robot icon 🤖
- 💎 Glass window opens
- ⚡ Ask questions
- 🎯 Get AI answers

---

## 🚨 Quick Fixes:

### Backend won't start?
```bash
# Check if port 8000 is busy
netstat -ano | findstr :8000
# Kill process: taskkill /PID <number> /F
```

### Frontend won't start?
```bash
# Check if port 3002 is busy
netstat -ano | findstr :3002
# Kill process: taskkill /PID <number> /F
```

### Chatbot not showing?
1. Login first! 🔐
2. Look bottom-right corner 👀
3. Check backend running ✅

---

## 📁 Important Files:

| File | Purpose |
|------|---------|
| `START_HERE.bat` | 🚀 Auto-start everything |
| `start-backend.bat` | ⚙️ Backend only |
| `start-frontend.bat` | 🎨 Frontend only |
| `SETUP_GUIDE.md` | 📖 Detailed guide |
| `QUICK_START.md` | ⚡ This file! |

---

## 🎉 That's It!

**Happy Learning! 🤖✨**

---

## 📞 URLs:

- **Frontend:** http://localhost:3002
- **Backend API:** http://127.0.0.1:8000
- **API Docs:** http://127.0.0.1:8000/docs
- **Health Check:** http://127.0.0.1:8000/health

---

**Made with ❤️ for Physical AI & Humanoid Robotics**
