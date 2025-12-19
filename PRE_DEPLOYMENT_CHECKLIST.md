# Pre-Deployment Checklist ✅

## Before You Push to GitHub

### 1. Check Sensitive Files
Run this command to make sure `.env` is NOT staged:
```bash
git status
```

**❌ If you see this (BAD):**
```
modified:   backend/.env
```

**✅ You should see this (GOOD):**
```
# .env should NOT appear in the list
```

---

### 2. Verify .gitignore
Your `.gitignore` already includes:
- ✅ `.env` and `.env.local`
- ✅ `venv/` and `.venv/`
- ✅ `__pycache__/`
- ✅ `node_modules/`
- ✅ `*.log`

---

### 3. Update Frontend API URL

**File:** `Robotic book/src/utils/auth.ts` (Line 10)

**BEFORE deployment:** Placeholder URL
```typescript
? 'https://your-backend.onrender.com'
```

**AFTER you deploy backend to Render:** Real URL
```typescript
? 'https://physical-ai-backend.onrender.com'  // Your actual Render URL
```

⚠️ **Important:** You'll update this AFTER deploying backend, then redeploy frontend.

---

### 4. Files Ready for Deployment

#### Backend Files ✅
- `backend/app.py` - CORS configured
- `render.yaml` - Render config
- `backend/.env.example` - Environment template
- `backend/requirements.txt` - Dependencies with gunicorn
- `backend/.env` - **NOT pushed to GitHub** (stays local)

#### Frontend Files ✅
- `Robotic book/src/utils/auth.ts` - API URL ready
- `Robotic book/docusaurus.config.ts` - GitHub Pages config

---

### 5. Environment Variables to Add in Render

From your `.env` file, copy these values:

| Variable | Get From |
|----------|----------|
| `GOOGLE_API_KEY` | Your `.env` |
| `QDRANT_URL` | Your `.env` |
| `QDRANT_API_KEY` | Your `.env` |
| `DATABASE_URL` | Your `.env` |
| `SECRET_KEY` | Your `.env` |
| `ENVIRONMENT` | Type: `production` |
| `ALLOWED_ORIGINS` | `https://salam747.github.io` |

---

### 6. Check Current Branch
```bash
git branch
```

Should show: `* 006-rag-backend`

If not, switch:
```bash
git checkout 006-rag-backend
```

---

## Deployment Steps (Quick Reference)

### Step 1: Deploy Backend First
1. Go to https://render.com
2. Sign in with GitHub
3. Create new Web Service
4. Select your repository
5. Set Root Directory: `backend`
6. Add environment variables
7. Deploy
8. Copy your Render URL

### Step 2: Update Frontend with Backend URL
1. Edit `Robotic book/src/utils/auth.ts`
2. Replace placeholder with real Render URL
3. Commit changes

### Step 3: Push & Deploy Frontend
```bash
git add .
git commit -m "feat: Configure Render deployment for backend"
git push origin 006-rag-backend
cd "Robotic book"
npm run deploy
```

---

## Quick Test Commands

### Test Backend Locally (Before Deployment)
```bash
cd backend
uvicorn app:app --reload
# Visit: http://127.0.0.1:8000/health
```

### Test Frontend Locally
```bash
cd "Robotic book"
npm start
# Visit: http://localhost:3001
```

### Test After Render Deployment
```
https://your-backend.onrender.com/health
```

Should return:
```json
{"status":"ok","message":"Server is healthy"}
```

---

## Final Checks Before Going Live

- [ ] Backend deployed to Render successfully
- [ ] Environment variables added in Render
- [ ] Frontend updated with real Render URL
- [ ] CORS includes GitHub Pages URL
- [ ] All tests passing locally
- [ ] No `.env` file in git status
- [ ] Committed and pushed all changes
- [ ] Frontend deployed to GitHub Pages

---

## Emergency Rollback

If something goes wrong:

### Rollback Frontend
```bash
git checkout main
cd "Robotic book"
npm run deploy
```

### Rollback Backend
1. Go to Render dashboard
2. Click your service
3. Go to "Events" tab
4. Find previous working deployment
5. Click "Rollback to this version"

---

## Need Help?

Check `DEPLOYMENT_GUIDE.md` for detailed step-by-step instructions.

Good luck! 🚀
