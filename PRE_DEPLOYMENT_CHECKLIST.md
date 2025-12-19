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
? 'https://your-vercel-backend.vercel.app'
```

**AFTER you deploy backend to Vercel:** Real URL
```typescript
? 'https://physical-ai-rag.vercel.app'  // Your actual Vercel URL
```

⚠️ **Important:** You'll update this AFTER deploying backend, then redeploy frontend.

---

### 4. Files Ready for Deployment

#### Backend Files ✅
- `backend/app.py` - CORS configured
- `backend/vercel.json` - Vercel config
- `backend/.vercelignore` - Ignore unnecessary files
- `backend/requirements.txt` - Dependencies
- `backend/.env` - **NOT pushed to GitHub** (stays local)

#### Frontend Files ✅
- `Robotic book/src/utils/auth.ts` - API URL ready
- `Robotic book/docusaurus.config.ts` - GitHub Pages config

---

### 5. Environment Variables to Add in Vercel

From your `.env` file, copy these values:

| Variable | Get From |
|----------|----------|
| `GOOGLE_API_KEY` | Your `.env` |
| `QDRANT_URL` | Your `.env` |
| `QDRANT_API_KEY` | Your `.env` |
| `DATABASE_URL` | Your `.env` |
| `SECRET_KEY` | Your `.env` |
| `ENVIRONMENT` | Type: `production` |

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
1. Go to https://vercel.com
2. Sign in with GitHub
3. Import your repository
4. Set Root Directory: `backend`
5. Add environment variables
6. Deploy
7. Copy your Vercel URL

### Step 2: Update Frontend with Backend URL
1. Edit `Robotic book/src/utils/auth.ts`
2. Replace placeholder with real Vercel URL
3. Commit changes

### Step 3: Push & Deploy Frontend
```bash
git add .
git commit -m "feat: Add backend deployment and connect frontend to Vercel API"
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

### Test After Vercel Deployment
```
https://YOUR-VERCEL-URL.vercel.app/health
```

Should return:
```json
{"status":"ok","message":"Server is healthy"}
```

---

## Final Checks Before Going Live

- [ ] Backend deployed to Vercel successfully
- [ ] Environment variables added in Vercel
- [ ] Frontend updated with real Vercel URL
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
1. Go to Vercel dashboard
2. Click "Deployments"
3. Find previous working deployment
4. Click "..." → "Promote to Production"

---

## Need Help?

Check `DEPLOYMENT_GUIDE.md` for detailed step-by-step instructions.

Good luck! 🚀
