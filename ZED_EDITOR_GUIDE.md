# 🎯 Zed Editor - Complete Guide

## 📋 Step-by-Step Instructions for Zed Editor

---

## 🚀 Method 1: Using Batch Files (EASIEST!)

### 1️⃣ Open File Explorer
```
Navigate to: D:\Q4-assignments\Physical_AI_and_Humanoid_Robotics_book
```

### 2️⃣ Double-Click
```
START_HERE.bat
```

### 3️⃣ Done!
- ✅ Backend starts automatically
- ✅ Frontend starts automatically
- ✅ Browser opens automatically
- ✅ Login with: test@example.com / Test@123

---

## 🛠️ Method 2: Using Zed Terminal

### Step 1: Open Project in Zed
```bash
# In terminal or cmd
cd D:\Q4-assignments\Physical_AI_and_Humanoid_Robotics_book
zed .
```

Or just open Zed and use: **File → Open Folder**

---

### Step 2: Open Zed Terminal

**Method A:** Keyboard Shortcut
- Press: **Ctrl + `** (backtick)
- Or: **Ctrl + J**

**Method B:** Menu
- Click: **Terminal → New Terminal**
- Or: **View → Terminal**

---

### Step 3: Start Backend (Terminal 1)

```bash
cd backend
uvicorn app:app --reload --port 8000
```

**Expected Output:**
```
INFO:     Uvicorn running on http://127.0.0.1:8000
INFO:     Application startup complete.
```

**Test it:**
- Open browser: http://127.0.0.1:8000/health
- Should see: `{"status":"ok","message":"Server is healthy"}`

---

### Step 4: Open New Terminal for Frontend

**Split Terminal:**
- Press: **Ctrl + Shift + `**
- Or: Click **+** button in terminal panel
- Or: Right-click terminal tab → **Split Terminal**

---

### Step 5: Start Frontend (Terminal 2)

```bash
cd "Robotic book"
npm start -- --port 3002
```

**Expected Output:**
```
[SUCCESS] Docusaurus website is running at: http://localhost:3002/
```

---

### Step 6: Open in Browser

**Option A:** Click the link in terminal
```
Ctrl + Click on: http://localhost:3002
```

**Option B:** Manually open
```
Browser → http://localhost:3002
```

---

## 🎨 Zed Terminal Features

### Terminal Layout Options:

**1. Split Horizontally:**
```
Terminal → Split Terminal Horizontally
```

**2. Split Vertically:**
```
Terminal → Split Terminal Vertically
```

**3. Multiple Tabs:**
```
Click + button to add new terminal tab
```

### Useful Shortcuts:

| Action | Shortcut |
|--------|----------|
| Open Terminal | `Ctrl + ` ` |
| New Terminal | `Ctrl + Shift + ` ` |
| Close Terminal | `Ctrl + W` |
| Next Terminal | `Ctrl + Tab` |
| Prev Terminal | `Ctrl + Shift + Tab` |
| Clear Terminal | `Ctrl + L` |
| Copy | `Ctrl + Shift + C` |
| Paste | `Ctrl + Shift + V` |

---

## 🎯 Recommended Zed Layout

### Layout 1: Side-by-Side
```
┌─────────────────┬─────────────────┐
│                 │                 │
│    Code Editor  │   File Tree     │
│                 │                 │
├─────────────────┴─────────────────┤
│  Terminal 1: Backend              │
├───────────────────────────────────┤
│  Terminal 2: Frontend             │
└───────────────────────────────────┘
```

### Layout 2: Stacked
```
┌───────────────────────────────────┐
│         Code Editor               │
├─────────────────┬─────────────────┤
│  Terminal 1:    │  Terminal 2:    │
│  Backend        │  Frontend       │
└─────────────────┴─────────────────┘
```

---

## 📝 Zed Terminal Commands Cheatsheet

### Navigate to Backend:
```bash
cd backend
```

### Navigate to Frontend:
```bash
cd "Robotic book"
```

### Check Python:
```bash
python --version
```

### Check Node:
```bash
node --version
npm --version
```

### Start Backend:
```bash
cd backend
uvicorn app:app --reload --port 8000
```

### Start Frontend:
```bash
cd "Robotic book"
npm start -- --port 3002
```

### Check Running Ports:
```bash
netstat -ano | findstr :8000
netstat -ano | findstr :3002
```

### Kill Process:
```bash
taskkill /PID <number> /F
```

---

## 🎨 What You'll See in Zed

### Terminal 1 (Backend):
```
INFO:     Will watch for changes in these directories: ['D:\\Q4-assignments\\Physical_AI_and_Humanoid_Robotics_book\\backend']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using WatchFiles
INFO:     Started server process [67890]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### Terminal 2 (Frontend):
```
> robotic-book@0.0.0 start
> docusaurus start --port 3002

[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3002/

webpack 5.103.0 compiled successfully
```

---

## 🐛 Troubleshooting in Zed

### Terminal Not Opening?
```
View → Toggle Terminal Panel
Or press: Ctrl + `
```

### Can't See Terminal Output?
```
Click on terminal panel to focus
Scroll up/down to see full output
```

### Wrong Directory?
```bash
# Check current directory
pwd

# Go back to root
cd D:\Q4-assignments\Physical_AI_and_Humanoid_Robotics_book
```

### Port Already in Use?
```bash
# Find what's using the port
netstat -ano | findstr :8000

# Kill the process
taskkill /PID <number> /F
```

### Backend Won't Start?
```bash
# Check Python installation
python --version

# Check if in correct directory
cd backend
dir  # Should see app.py
```

### Frontend Won't Start?
```bash
# Check Node installation
node --version

# Check if in correct directory
cd "Robotic book"
dir  # Should see package.json

# Install dependencies if needed
npm install
```

---

## 🎯 Pro Tips for Zed

### 1. Use Batch Files
Instead of typing commands manually, just run:
```
start-backend.bat
start-frontend.bat
```

### 2. Keep Terminals Open
Don't close terminals while servers are running.
Use `Ctrl + C` to stop servers first.

### 3. Use Terminal Tabs
Create separate tabs for:
- Backend
- Frontend
- Testing/Commands

### 4. Save Terminal Sessions
Zed may remember your terminal sessions when you reopen.

### 5. Quick Commands
Create aliases or scripts for frequently used commands.

---

## 📋 Quick Checklist

Before starting:
- [ ] Zed editor installed
- [ ] Python 3.8+ installed
- [ ] Node.js 16+ installed
- [ ] Project folder opened in Zed
- [ ] `.env` file configured in backend
- [ ] Internet connection for API calls

After starting:
- [ ] Backend running on port 8000
- [ ] Frontend running on port 3002
- [ ] Health check passing
- [ ] Browser opened to localhost:3002
- [ ] Can login with test credentials
- [ ] Chatbot icon visible (bottom-right)

---

## 🎉 Success Indicators

### Backend Started Successfully:
```
✅ "Application startup complete" message
✅ http://127.0.0.1:8000/health returns {"status":"ok"}
✅ No error messages in terminal
```

### Frontend Started Successfully:
```
✅ "Docusaurus website is running" message
✅ "compiled successfully" message
✅ Can open http://localhost:3002 in browser
```

### Everything Working:
```
✅ Homepage loads with beautiful design
✅ Can navigate to login page
✅ Can login with test@example.com
✅ See robot icon in bottom-right
✅ Chatbot opens and responds
```

---

## 💡 Remember

1. **Always start Backend FIRST**, then Frontend
2. **Keep both terminals open** while using the app
3. **Use Ctrl + C** to stop servers gracefully
4. **Check for errors** in terminal output
5. **Test backend health** before testing frontend

---

## 🚀 Quick Commands Summary

```bash
# Backend
cd backend && uvicorn app:app --reload --port 8000

# Frontend (new terminal)
cd "Robotic book" && npm start -- --port 3002

# Or just use:
START_HERE.bat
```

---

**Happy Coding in Zed! 🎯✨**

**Made with ❤️ for Zed Editor users**
