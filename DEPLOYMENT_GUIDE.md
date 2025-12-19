# Complete Deployment Guide - Physical AI Book

## Overview
This guide will help you deploy:
1. **Frontend** → GitHub Pages (Already done)
2. **Backend** → Vercel (New deployment)

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
✅ `backend/vercel.json` - Created
✅ `backend/.vercelignore` - Created
✅ `Robotic book/src/utils/auth.ts` - API URL configured
✅ `backend/app.py` - CORS configured

---

## Part 2: Deploy Backend to Vercel

### Step 1: Create Vercel Account
1. Go to https://vercel.com
2. Click "Sign Up"
3. Sign up with GitHub (easiest option)
4. Authorize Vercel to access your GitHub

### Step 2: Import Your Project
1. Click "Add New" → "Project"
2. Select "Import Git Repository"
3. Find your repository: `Physical-AI-and-Humanoid-Robotics-book`
4. Click "Import"

### Step 3: Configure Build Settings

**IMPORTANT:** Vercel ko batana hai ke backend folder use karna hai

```
Root Directory: backend
Framework Preset: Other
Build Command: (leave empty)
Output Directory: (leave empty)
Install Command: pip install -r requirements.txt
```

### Step 4: Add Environment Variables

Click "Environment Variables" and add these:

| Variable Name | Value | Where to Get |
|---------------|-------|--------------|
| `GOOGLE_API_KEY` | Your Gemini API key | From your `.env` file |
| `QDRANT_URL` | Your Qdrant URL | From your `.env` file |
| `QDRANT_API_KEY` | Your Qdrant API key | From your `.env` file |
| `DATABASE_URL` | Your Neon Postgres URL | From your `.env` file |
| `SECRET_KEY` | Your JWT secret | From your `.env` file |
| `ENVIRONMENT` | `production` | Type manually |

**How to add:**
1. Click "Add" for each variable
2. Paste the value from your `.env` file
3. Click "Add" to confirm

### Step 5: Deploy
1. Click "Deploy"
2. Wait 2-3 minutes for build
3. You'll get a URL like: `https://your-project-name.vercel.app`

---

## Part 3: Update Frontend with Backend URL

### Step 1: Get Your Vercel Backend URL
After deployment, copy your Vercel URL (e.g., `https://physical-ai-backend.vercel.app`)

### Step 2: Update Frontend Code

Open `Robotic book/src/utils/auth.ts` and update line 10:

```typescript
export const API_URL = process.env.NODE_ENV === 'production'
    ? 'https://YOUR-VERCEL-URL.vercel.app'  // ← Update this
    : 'http://127.0.0.1:8000';
```

Replace `YOUR-VERCEL-URL` with your actual Vercel URL.

### Step 3: Update Vercel CORS Settings

Add your GitHub Pages URL to Vercel environment variables:

```
Variable: ALLOWED_ORIGINS
Value: https://salam747.github.io
```

### Step 4: Redeploy Vercel Backend
After adding `ALLOWED_ORIGINS`:
1. Go to Vercel dashboard
2. Click "Deployments"
3. Click "Redeploy" on latest deployment

---

## Part 4: Push & Deploy Frontend

### Step 1: Commit Changes
```bash
cd "D:\Q4-assignments\Physical_AI_and_Humanoid_Robotics_book"
git add .
git commit -m "feat: Add backend deployment configuration and update API URLs"
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
Visit: `https://YOUR-VERCEL-URL.vercel.app/health`

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
1. Check Vercel backend is running: Visit `/health` endpoint
2. Check CORS is configured correctly
3. Check browser console for exact error

### Problem: Backend deployment failed

**Solution:**
1. Check `requirements.txt` has all dependencies
2. Check Python version (should be 3.11)
3. Check Vercel logs for error details

### Problem: Authentication not working

**Solution:**
1. Check `SECRET_KEY` is set in Vercel
2. Check `DATABASE_URL` is correct
3. Check Neon Postgres database is accessible

### Problem: Chatbot says "couldn't find relevant information"

**Solution:**
1. You need to ingest data first
2. Visit: `https://YOUR-VERCEL-URL.vercel.app/ingest`
3. Wait for ingestion to complete
4. This loads book content into Qdrant

---

## Important Notes

### ⚠️ First-Time Setup on Vercel
After deployment, run the ingestion endpoint ONCE:
```
POST https://YOUR-VERCEL-URL.vercel.app/ingest
```

This loads all book content into Qdrant.

### 🔒 Security
- Never commit `.env` file to GitHub
- Keep your API keys secret
- Use Vercel environment variables for production

### 💰 Free Tier Limits
- **Vercel:** 100GB bandwidth/month (plenty for your use)
- **Qdrant:** Free tier available
- **Neon:** Free tier: 10GB storage
- **Gemini:** Free tier: 60 requests/minute

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
3. ✅ Monitor Vercel logs for errors
4. ✅ Update README with live URL

---

## Support URLs

- **Vercel Dashboard:** https://vercel.com/dashboard
- **GitHub Repository:** https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book
- **Live Frontend:** https://salam747.github.io/Physical-AI-and-Humanoid-Robotics-book/
- **Backend API:** https://YOUR-PROJECT.vercel.app

---

Good luck with deployment! 🚀
