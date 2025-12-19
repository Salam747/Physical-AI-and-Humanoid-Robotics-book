# Render Quick Start - 5 Minute Setup

## Kya karna hai? (Quick Summary)

### 1. Files Ready Hain ✅
- ✅ `render.yaml` - Already created
- ✅ `backend/requirements.txt` - Already exists
- ✅ `.env.example` - Template ready

### 2. Abhi Kya Karna Hai?

#### A. GitHub pe Push Karo

```bash
# Check current status
git status

# Add new files
git add render.yaml RENDER_DEPLOYMENT_GUIDE.md RENDER_QUICK_START.md backend/.env.example

# Commit
git commit -m "feat: Add Render deployment configuration and guides"

# Push
git push origin 006-rag-backend
```

#### B. Render pe Jao

1. https://render.com pe jao
2. GitHub se sign up karo
3. Dashboard → **"New +"** → **"Blueprint"**
4. Repository select karo: `Physical_AI_and_Humanoid_Robotics_book`
5. Branch: `006-rag-backend`
6. **"Apply"** click karo

#### C. Environment Variables Set Karo

Service settings → Environment tab → Add karo:

```env
GOOGLE_API_KEY=your_gemini_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_database_url
SECRET_KEY=your_random_secret_key
ALLOWED_ORIGINS=https://salam747.github.io
```

**Important:** Ye values apne actual credentials se replace karo!

#### D. Deploy!

- Render automatically deploy karega
- 5-10 minutes wait karo
- URL milega: `https://physical-ai-book-backend.onrender.com`

#### E. Test Karo

Browser mein:
```
https://physical-ai-book-backend.onrender.com/health
```

Response:
```json
{"status": "ok", "message": "Server is healthy"}
```

---

## Vercel Files Kya Karein?

### Option 1: Delete Karo (Clean approach)

```bash
rm vercel.json
rm VERCEL_BEGINNER_GUIDE.md
git add .
git commit -m "chore: Remove Vercel configuration (migrated to Render)"
git push
```

### Option 2: Keep Karo (Backup ke liye)

```bash
# Bas ignore karo, Render use nahi karega
# .gitignore mein add karo (optional):
echo "vercel.json" >> .gitignore
```

**Recommendation:** Option 1 (delete) because clean codebase

---

## Frontend Update

**File:** `Robotic book/src/components/RagChatbot.jsx`

Find:
```javascript
const API_URL = 'https://your-old-api.vercel.app';
```

Replace with:
```javascript
const API_URL = process.env.NODE_ENV === 'production'
  ? 'https://physical-ai-book-backend.onrender.com'
  : 'http://localhost:8000';
```

Then deploy frontend:
```bash
cd "Robotic book"
npm run build
npm run deploy
```

---

## Troubleshooting Quick Fixes

### Problem: Build failing
```bash
# Check requirements.txt
cat backend/requirements.txt

# Test locally
cd backend
pip install -r requirements.txt
uvicorn app:app --reload
```

### Problem: Environment variables missing
1. Go to Render dashboard
2. Your service → Environment tab
3. Add all required variables (see section C above)
4. Click "Save Changes"

### Problem: Database connection error
- Check `DATABASE_URL` format:
  ```
  postgresql://username:password@host:port/database_name
  ```
- Make sure Neon database is active and accessible

### Problem: CORS error
- Add frontend URL to `ALLOWED_ORIGINS`:
  ```
  ALLOWED_ORIGINS=https://salam747.github.io,http://localhost:3000
  ```

---

## Summary

**Changes Required:**
1. ✅ Created: `render.yaml`
2. ✅ Created: `.env.example`
3. ✅ Created: Deployment guides
4. ⏳ TODO: Push to GitHub
5. ⏳ TODO: Deploy on Render
6. ⏳ TODO: Update frontend API URL
7. ⏳ TODO: (Optional) Remove Vercel files

**Estimated Time:** 10-15 minutes total

**Cost:** FREE (with 750 hours/month on Render free tier)

---

## Next Steps After Deployment

1. Test all endpoints:
   - `/health`
   - `/query`
   - `/auth/register`
   - `/auth/login`

2. Check logs for any errors:
   - Render dashboard → Logs tab

3. Monitor performance:
   - Response times
   - Memory usage
   - Any crashes

4. Setup monitoring (optional):
   - UptimeRobot to prevent sleep mode
   - Error tracking with Sentry

---

**Need Help? Check `RENDER_DEPLOYMENT_GUIDE.md` for detailed instructions!**
