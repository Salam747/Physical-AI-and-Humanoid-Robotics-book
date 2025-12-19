# Vercel Deployment Guide for Beginners 🚀

## What is Vercel?

Vercel is like a **free hosting service** for your backend. Think of it like this:
- Your **frontend** (website) lives on GitHub Pages
- Your **backend** (API server) will live on Vercel
- They talk to each other through the internet

---

## Step-by-Step: Deploy Backend to Vercel

### Step 1: Create Account (5 minutes)

1. **Go to Vercel website:**
   - Open browser: https://vercel.com

2. **Sign Up with GitHub:**
   - Click "Sign Up"
   - Click "Continue with GitHub"
   - GitHub will ask permission - Click "Authorize Vercel"
   - ✅ Done! You're logged in

---

### Step 2: Push Your Code to GitHub First

Before deploying to Vercel, make sure your latest code is on GitHub:

```bash
# Navigate to project
cd "D:\Q4-assignments\Physical_AI_and_Humanoid_Robotics_book"

# Check what files changed
git status

# Add all changes
git add .

# Commit with message
git commit -m "feat: Ready for Vercel deployment"

# Push to GitHub
git push origin 006-rag-backend
```

**⚠️ Important:** Make sure `.env` is NOT in the list when you run `git status`

---

### Step 3: Import Project to Vercel (10 minutes)

1. **In Vercel Dashboard:**
   - Click "Add New" button (top right)
   - Select "Project"

2. **Import from GitHub:**
   - Click "Import Git Repository"
   - Find: `Physical-AI-and-Humanoid-Robotics-book`
   - Click "Import" next to it

3. **Configure Project Settings:**

   **Project Name:** (Vercel will suggest one, you can change it)
   ```
   physical-ai-rag-backend
   ```

   **Framework Preset:**
   ```
   Other
   ```

   **Root Directory:** (Very Important!)
   ```
   Click "Edit" → Select "backend"
   ```

   **Build Settings:**
   - Build Command: (leave empty)
   - Output Directory: (leave empty)
   - Install Command: `pip install -r requirements.txt`

---

### Step 4: Add Environment Variables (5 minutes)

**This is the most important step!** Your backend needs these secrets to work.

1. **Scroll down to "Environment Variables"**

2. **Open your `.env` file locally:**
   ```
   D:\Q4-assignments\Physical_AI_and_Humanoid_Robotics_book\backend\.env
   ```

3. **Copy each value and add to Vercel:**

   **Add Variable 1:**
   - Name: `GOOGLE_API_KEY`
   - Value: (Copy from your .env file)
   - Click "Add"

   **Add Variable 2:**
   - Name: `QDRANT_URL`
   - Value: (Copy from your .env file)
   - Click "Add"

   **Add Variable 3:**
   - Name: `QDRANT_API_KEY`
   - Value: (Copy from your .env file)
   - Click "Add"

   **Add Variable 4:**
   - Name: `DATABASE_URL`
   - Value: (Copy from your .env file)
   - Click "Add"

   **Add Variable 5:**
   - Name: `SECRET_KEY`
   - Value: (Copy from your .env file)
   - Click "Add"

   **Add Variable 6:**
   - Name: `ENVIRONMENT`
   - Value: `production` (type this manually)
   - Click "Add"

   **Add Variable 7:**
   - Name: `ALLOWED_ORIGINS`
   - Value: `https://salam747.github.io`
   - Click "Add"

---

### Step 5: Deploy! (2-3 minutes)

1. **Click the big "Deploy" button**

2. **Wait for deployment:**
   - You'll see a progress screen
   - Building... (1-2 minutes)
   - Deploying... (30 seconds)
   - ✅ Success!

3. **Copy your URL:**
   - You'll see: `https://your-project-name.vercel.app`
   - **Copy this URL** - you'll need it next!

---

### Step 6: Test Your Backend

1. **Open in browser:**
   ```
   https://your-project-name.vercel.app/health
   ```

2. **You should see:**
   ```json
   {"status":"ok","message":"Server is healthy"}
   ```

3. **If you see this:** ✅ Backend is working!
4. **If error:** Check Step 7 (Troubleshooting)

---

### Step 7: Update Frontend with Backend URL

Now tell your frontend where the backend is:

1. **Open this file:**
   ```
   Robotic book/src/utils/auth.ts
   ```

2. **Find line 10:**
   ```typescript
   ? 'https://your-vercel-backend.vercel.app'
   ```

3. **Replace with YOUR real URL:**
   ```typescript
   ? 'https://physical-ai-rag-backend.vercel.app'  // Your URL here
   ```

4. **Save the file**

---

### Step 8: Redeploy Frontend (2 minutes)

```bash
# Commit the change
git add .
git commit -m "feat: Connect frontend to Vercel backend"
git push origin 006-rag-backend

# Deploy to GitHub Pages
cd "Robotic book"
npm run deploy
```

---

### Step 9: First-Time Backend Setup (5 minutes)

**Important:** You need to load the book content into Qdrant.

**Method 1: Using Browser**
1. Open: `https://YOUR-VERCEL-URL.vercel.app/docs`
2. Find `/ingest` endpoint
3. Click "Try it out"
4. Click "Execute"
5. Wait 2-3 minutes

**Method 2: Using Command**
```bash
curl -X POST "https://YOUR-VERCEL-URL.vercel.app/ingest"
```

This loads all book content. Only do this ONCE.

---

## Testing Everything

### Test 1: Backend Health
```
Visit: https://YOUR-VERCEL-URL.vercel.app/health
Expected: {"status":"ok","message":"Server is healthy"}
```

### Test 2: Frontend Login
```
1. Visit: https://salam747.github.io/Physical-AI-and-Humanoid-Robotics-book/
2. Click "Login"
3. Try to sign up/login
4. Should work!
```

### Test 3: Chatbot
```
1. Open chatbot (bottom right)
2. Ask: "What is ROS 2?"
3. Should get response from book
```

---

## Common Issues & Solutions

### Issue 1: "Build Failed" in Vercel

**Check:**
- Did you select "backend" as Root Directory?
- Is `requirements.txt` in the backend folder?

**Fix:**
1. Go to Project Settings
2. Click "General"
3. Change Root Directory to "backend"
4. Redeploy

---

### Issue 2: "Internal Server Error"

**Check Vercel Logs:**
1. Go to Vercel Dashboard
2. Click your project
3. Click "Deployments"
4. Click latest deployment
5. Click "Functions" tab
6. Look for red errors

**Common Causes:**
- Missing environment variable
- Wrong database URL
- API key not working

---

### Issue 3: Frontend can't connect to backend

**Error in browser console:**
```
Failed to fetch
CORS error
```

**Fix:**
1. Go to Vercel Dashboard
2. Settings → Environment Variables
3. Add: `ALLOWED_ORIGINS` = `https://salam747.github.io`
4. Redeploy

---

### Issue 4: Chatbot says "couldn't find information"

**Cause:** Book content not loaded yet

**Fix:**
Visit: `https://YOUR-VERCEL-URL.vercel.app/ingest`

Wait for it to complete. Do this only once.

---

## Vercel Dashboard Guide

### View Logs
1. Dashboard → Your Project
2. Deployments → Latest
3. Functions → Click function name
4. See real-time logs

### Update Environment Variables
1. Dashboard → Your Project
2. Settings → Environment Variables
3. Edit or add new ones
4. **Must redeploy** after changes

### Redeploy
1. Dashboard → Your Project
2. Deployments
3. Click "..." on latest
4. Click "Redeploy"

---

## Free Tier Limits

**Vercel Free Plan:**
- ✅ 100GB bandwidth/month (plenty!)
- ✅ Unlimited projects
- ✅ Automatic HTTPS
- ✅ Automatic deployments from GitHub

**What happens if you exceed?**
- Site will keep working
- Vercel may ask you to upgrade
- Your usage is very low, so no worries!

---

## Updating Your Deployed Backend

### Option 1: Automatic (Recommended)
Just push to GitHub:
```bash
git add .
git commit -m "update: Fixed bug"
git push origin 006-rag-backend
```
Vercel automatically redeploys! ✨

### Option 2: Manual
1. Go to Vercel Dashboard
2. Click "Deployments"
3. Click "Redeploy" on latest

---

## Important URLs to Bookmark

| Service | URL | Purpose |
|---------|-----|---------|
| Vercel Dashboard | https://vercel.com/dashboard | Manage backend |
| GitHub Repo | https://github.com/Salam747/Physical-AI-and-Humanoid-Robotics-book | Code |
| Live Frontend | https://salam747.github.io/Physical-AI-and-Humanoid-Robotics-book/ | Website |
| Backend API | https://YOUR-PROJECT.vercel.app | API |
| Backend Docs | https://YOUR-PROJECT.vercel.app/docs | API documentation |

---

## Next Steps After Deployment

1. ✅ Test all features (login, chatbot, etc.)
2. ✅ Share your live URL with friends/teachers
3. ✅ Update README.md with live links
4. ✅ Monitor Vercel dashboard for any errors
5. ✅ Celebrate! 🎉

---

## Quick Command Reference

```bash
# Check git status
git status

# Add and commit
git add .
git commit -m "your message"

# Push to GitHub
git push origin 006-rag-backend

# Deploy frontend
cd "Robotic book"
npm run deploy

# Test backend locally
cd backend
uvicorn app:app --reload
```

---

## Need More Help?

- **Detailed Guide:** Read `DEPLOYMENT_GUIDE.md`
- **Pre-deployment Checks:** Read `PRE_DEPLOYMENT_CHECKLIST.md`
- **Vercel Docs:** https://vercel.com/docs
- **FastAPI + Vercel:** https://vercel.com/docs/frameworks/python

---

**You got this! 💪 Vercel makes deployment easy!**
