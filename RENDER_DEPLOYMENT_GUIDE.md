# Render Deployment Guide - Urdu/English

## Render pe Deploy karne ka Complete Tareeqa

### Prerequisites (Zaroori Cheezein)

1. **Render Account** - https://render.com pe signup karen
2. **GitHub Repository** - Aapka code GitHub pe pushed hona chahiye
3. **Environment Variables** - Neeche diye gaye credentials ready rakhen:
   - `GOOGLE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `DATABASE_URL` (agar Render ka database use nahi kar rahe)

---

## Step-by-Step Deployment Process

### Step 1: Render Dashboard Access

1. https://render.com pe jao
2. **Sign up** ya **Log in** karo
3. **Dashboard** pe jao

### Step 2: New Web Service Banao

#### Option A: render.yaml se Deploy (Recommended)

1. Dashboard pe **"New +"** button click karo
2. **"Blueprint"** select karo
3. Apna GitHub repository connect karo
4. Branch select karo: `006-rag-backend`
5. Render automatically `render.yaml` detect kar lega
6. **"Apply"** button click karo

#### Option B: Manual Setup

1. Dashboard pe **"New +"** button click karo
2. **"Web Service"** select karo
3. GitHub repository connect karo
4. Configuration fill karo:
   - **Name:** `physical-ai-book-backend`
   - **Region:** Singapore (ya apne paas ka)
   - **Branch:** `006-rag-backend`
   - **Root Directory:** `backend`
   - **Runtime:** Python 3
   - **Build Command:** `pip install -r requirements.txt`
   - **Start Command:** `uvicorn app:app --host 0.0.0.0 --port $PORT`
   - **Plan:** Free

### Step 3: Environment Variables Add Karo

1. Service settings mein **"Environment"** tab pe jao
2. Ye variables add karo:

```
GOOGLE_API_KEY=<your_gemini_api_key>
QDRANT_URL=<your_qdrant_url>
QDRANT_API_KEY=<your_qdrant_api_key>
DATABASE_URL=<your_database_url>
SECRET_KEY=<random_secret_key>
ALLOWED_ORIGINS=https://salam747.github.io,http://localhost:3000
```

**Important Notes:**
- `SECRET_KEY` random honi chahiye (Render auto-generate kar sakta hai)
- `DATABASE_URL` format: `postgresql://user:password@host:port/dbname`
- Agar Render ka built-in database use kar rahe ho, to yeh automatically set ho jayega

### Step 4: Database Setup (Optional)

#### Agar Render ka PostgreSQL Database Chahiye:

1. Dashboard pe **"New +"** → **"PostgreSQL"** select karo
2. Configuration:
   - **Name:** `physical-ai-book-db`
   - **Database Name:** `physical_ai_book`
   - **Region:** Same as web service
   - **Plan:** Free
3. Create karne ke baad, **Internal Database URL** copy karo
4. Web service ke environment variables mein `DATABASE_URL` add karo

#### Agar Existing Neon Database Use Kar Rahe Ho:

1. Neon dashboard se connection string copy karo
2. Render mein `DATABASE_URL` environment variable set karo

### Step 5: Deploy!

1. **"Create Web Service"** button click karo
2. Render automatically build aur deploy karega
3. Logs mein dekhte raho - deployment 5-10 minutes le sakti hai
4. Build complete hone ke baad, aapko service ka URL milega:
   - Example: `https://physical-ai-book-backend.onrender.com`

### Step 6: Health Check Test Karo

1. Browser mein jao: `https://your-app-name.onrender.com/health`
2. Response milna chahiye:
```json
{
  "status": "ok",
  "message": "Server is healthy"
}
```

### Step 7: Frontend Update Karo

Frontend mein API URL update karna hoga:

**File:** `Robotic book/src/components/RagChatbot.jsx`

```javascript
// Old (Vercel):
const API_URL = 'https://your-vercel-app.vercel.app';

// New (Render):
const API_URL = 'https://physical-ai-book-backend.onrender.com';
```

### Step 8: Frontend Deploy Karo (GitHub Pages)

```bash
cd "Robotic book"
npm run build
npm run deploy
```

---

## Troubleshooting (Masail aur Hal)

### Issue 1: Build Fail Ho Raha Hai

**Possible Reasons:**
- `requirements.txt` mein missing dependencies
- Python version mismatch

**Solution:**
```bash
# Local test karo:
cd backend
pip install -r requirements.txt
```

### Issue 2: App Crash Ho Raha Hai

**Check karo:**
1. Logs mein errors dekho (Render dashboard → Logs tab)
2. Environment variables sahi hain ya nahi
3. Database connection string correct hai ya nahi

**Common Error:**
```
ModuleNotFoundError: No module named 'X'
```
**Solution:** `requirements.txt` mein wo module add karo

### Issue 3: CORS Error

**Error:**
```
Access to fetch at 'https://...' from origin 'https://salam747.github.io' has been blocked by CORS policy
```

**Solution:**
`ALLOWED_ORIGINS` environment variable mein frontend URL add karo:
```
ALLOWED_ORIGINS=https://salam747.github.io
```

### Issue 4: Database Connection Failed

**Check:**
1. `DATABASE_URL` format sahi hai?
   ```
   postgresql://user:password@host:port/database_name
   ```
2. Database accessible hai?
3. Neon/Render database active hai?

### Issue 5: Free Tier Sleep Mode

**Issue:** Render free tier mein service 15 minutes inactivity ke baad sleep mode mein chali jati hai

**Solutions:**
1. **Cron Job:** UptimeRobot ya cron-job.org use karke har 10 minutes mein health check ping karo
2. **Paid Plan:** $7/month se always-on service milegi

---

## Free Tier Limitations

| Feature | Limit |
|---------|-------|
| Build time | 90 seconds (usually sufficient) |
| Monthly build minutes | 400 minutes |
| Bandwidth | 100 GB/month |
| Sleep after inactivity | 15 minutes |
| Cold start time | ~30 seconds |

**Tip:** Agar cold starts issue hai, to paid plan consider karo ($7/month)

---

## Deployment Checklist

Before deploying, make sure:
- [ ] `render.yaml` file created
- [ ] `.env.example` file created
- [ ] All environment variables ready
- [ ] GitHub repository updated and pushed
- [ ] Database (Neon/Render) accessible
- [ ] Qdrant vector database running
- [ ] Google Gemini API key valid
- [ ] Frontend API URL updated

---

## Post-Deployment

### Monitor Your App:

1. **Logs:** Render dashboard → Your service → Logs
2. **Metrics:** CPU, Memory usage dekh sakte ho
3. **Events:** Deployment history

### Keep Updated:

```bash
# Code update karne ke baad:
git add .
git commit -m "Update: your changes"
git push origin 006-rag-backend

# Render automatically redeploy karega
```

---

## Cost Comparison

| Platform | Free Tier | Paid (Basic) |
|----------|-----------|--------------|
| **Vercel** | Serverless (limited space) | $20/month |
| **Render** | 750 hours/month | $7/month (always-on) |
| **Heroku** | No free tier | $7/month |
| **Railway** | $5 credit/month | Pay as you go |

**Recommendation:** Render ka free tier achha hai for starting, later $7/month plan sufficient hai for small traffic

---

## Additional Resources

- **Render Docs:** https://render.com/docs
- **Python Deployment:** https://render.com/docs/deploy-fastapi
- **Database Guide:** https://render.com/docs/databases

---

## Need Help?

**Common Commands:**

```bash
# Check if app is accessible
curl https://your-app-name.onrender.com/health

# Test query endpoint
curl -X POST https://your-app-name.onrender.com/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is physical AI?"}'
```

**Support:**
- Render Community: https://community.render.com
- Discord: https://discord.gg/render

---

**Good Luck with Deployment! 🚀**
