# Complete Deployment Guide - Physical AI Book

## Overview
This guide will help you deploy:
1. **Frontend** → GitHub Pages (Already done)
2. **Backend** → Render (Free deployment)

---

## Part 1: Prepare for Deployment

### 1.1 Check .gitignore
Make sure these files are NOT pushed to GitHub:

```bash
# Backend
backend/.env
backend/venv/
backend/__pycache__/
backend/*.pyc
backend/backend.log

# Frontend
Robotic book/node_modules/
Robotic book/.docusaurus/
Robotic book/build/
```

### 1.2 Files Ready for Deployment
✅ `render.yaml` - Created
✅ `backend/.env.example` - Environment template
✅ `Robotic book/src/utils/auth.ts` - API URL configured
✅ `backend/app.py` - CORS configured

---

## Part 2: Deploy Backend to Render

### Step 1: Create Render Account
1. Go to https://render.com
2. Click "Get Started for Free"
3. Sign up with GitHub (easiest option)
4. Authorize Render to access your GitHub

### Step 2: Create New Web Service
1. Click "New" → "Web Service"
2. Connect your GitHub repository: `Physical-AI-and-Humanoid-Robotics-book`
3. Select the repository
4. Click "Connect"

### Step 3: Configure Build Settings

```
Name: physical-ai-backend
Region: Singapore (or closest to you)
Branch: 006-rag-backend
Root Directory: backend
Runtime: Python 3
Build Command: pip install -r requirements.txt
Start Command: gunicorn app:app --bind 0.0.0.0:$PORT
Instance Type: Free
```

### Step 4: Add Environment Variables

Click "Environment" and add these:

| Variable Name | Value | Where to Get |
|---------------|-------|--------------|
| `GOOGLE_API_KEY` | Your Gemini API key | From your `.env` file |
| `QDRANT_URL` | Your Qdrant URL | From your `.env` file |
| `QDRANT_API_KEY` | Your Qdrant API key | From your `.env` file |
| `DATABASE_URL` | Your Neon Postgres URL | From your `.env` file |
| `SECRET_KEY` | Your JWT secret | From your `.env` file |
| `ENVIRONMENT` | `production` | Type manually |
| `ALLOWED_ORIGINS` | `https://salam747.github.io` | Your GitHub Pages URL |

**How to add:**
1. Click "Add Environment Variable"
2. Enter key and value
3. Click "Save"

### Step 5: Deploy
1. Click "Create Web Service"
2. Wait 3-5 minutes for build
3. You'll get a URL like: `https://physical-ai-backend.onrender.com`

---

## Part 3: Update Frontend with Backend URL

### Step 1: Get Your Render Backend URL
After deployment, copy your Render URL (e.g., `https://physical-ai-backend.onrender.com`)

### Step 2: Update Frontend Code

Open `Robotic book/src/utils/auth.ts` and update line 10:

```typescript
export const API_URL = process.env.NODE_ENV === 'production'
    ? 'https://your-backend.onrender.com'  // ← Update this
    : 'http://127.0.0.1:8000';
```

Replace with your actual Render URL.

### Step 3: Update CORS Settings
Your `ALLOWED_ORIGINS` environment variable in Render should already include your GitHub Pages URL:
```
https://salam747.github.io
```

If you need to add more origins, separate them with commas in Render dashboard.

---

## Part 4: Push & Deploy Frontend

### Step 1: Commit Changes
```bash
cd "D:\Q4-assignments\Physical_AI_and_Humanoid_Robotics_book"
git add .
git commit -m "feat: Configure Render deployment for backend"
```

### Step 2: Push to GitHub
```bash
git push origin 006-rag-backend
```

### Step 3: Deploy to GitHub Pages
```bash
cd "Robotic book"
npm run deploy
```

---

## Part 5: Test Everything

### Test 1: Backend Health Check
Visit: `https://your-backend.onrender.com/health`

Expected response:
```json
{"status":"ok","message":"Server is healthy"}
```

### Test 2: Frontend Authentication
1. Visit: https://salam747.github.io/Physical-AI-and-Humanoid-Robotics-book/
2. Click on Login
3. Try to login/signup
4. Check if it works

### Test 3: Chatbot
1. Open chatbot
2. Ask: "What is ROS 2?"
3. Check if you get a response

---

## Troubleshooting

### Problem: "Failed to fetch" error in chatbot

**Solution:**
1. Check Render backend is running: Visit `/health` endpoint
2. Check CORS is configured correctly in environment variables
3. Check browser console for exact error
4. Note: Render free tier may sleep after 15 minutes of inactivity (first request takes ~30 seconds to wake up)

### Problem: Backend deployment failed

**Solution:**
1. Check `requirements.txt` has all dependencies
2. Check Python version (should be 3.11)
3. Check Render logs for error details
4. Verify all environment variables are set

### Problem: Authentication not working

**Solution:**
1. Check `SECRET_KEY` is set in Render
2. Check `DATABASE_URL` is correct
3. Check Neon Postgres database is accessible

### Problem: Chatbot says "couldn't find relevant information"

**Solution:**
1. You need to ingest data first
2. Visit: `https://your-backend.onrender.com/ingest`
3. Wait for ingestion to complete
4. This loads book content into Qdrant

---

## Important Notes

### ⚠️ First-Time Setup on Render
After deployment, run the ingestion endpoint ONCE:
```
POST https://your-backend.onrender.com/ingest
```

This loads all book content into Qdrant.

### 🔒 Security
- Never commit `.env` file to GitHub
- Keep your API keys secret
- Use Render environment variables for production

### 💰 Free Tier Limits
- **Render:** Free web service with 750 hours/month (sleeps after 15 min inactivity)
- **Qdrant:** Free tier available
- **Neon:** Free tier: 10GB storage
- **Gemini:** Free tier: 60 requests/minute

### ⏰ Render Free Tier Sleep
Free services sleep after 15 minutes of inactivity. First request after sleep takes ~30 seconds to wake up.

---

## Quick Commands Reference

### Backend Local Development
```bash
cd backend
uvicorn app:app --reload
```

### Frontend Local Development
```bash
cd "Robotic book"
npm start
```

### Deploy Frontend to GitHub Pages
```bash
cd "Robotic book"
npm run deploy
```

### Git Commands
```bash
# Check status
git status

# Stage all changes
git add .

# Commit
git commit -m "your message"

# Push to GitHub
git push origin 006-rag-backend
```

---

## Next Steps After Deployment

1. ✅ Test all features on production
2. ✅ Share the live URL with others
3. ✅ Monitor Render logs for errors
4. ✅ Update README with live URL

---

## Support URLs

- **Render Dashboard:** https://dashboard.render.com
- **GitHub Repository:** https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book
- **Live Frontend:** https://salam747.github.io/Physical-AI-and-Humanoid-Robotics-book/
- **Backend API:** https://your-backend.onrender.com

---

Good luck with deployment! 🚀
