# Backend Deployment Guide - Step by Step

## Overview
This guide will help you deploy your RAG chatbot backend to **Render.com** (free tier) without changing any code in your project.

---

## Prerequisites Checklist

Before starting, ensure you have:
- ✅ GitHub account
- ✅ Render.com account (create free at https://render.com)
- ✅ Your API keys ready:
  - GEMINI_API_KEY: `AIzaSyAnTWntUl05EyTX_RSJsaPIxeIWd3fiUyw`
  - COHERE_API_KEY: `0DRQcyTI98p3HRpjuQv8tvg4IcQtRVBeublwiHoe`
  - QDRANT_URL: `https://6eb3cc7d-3f4e-46a5-ae7c-20d8d583c238.europe-west3-0.gcp.cloud.qdrant.io:6333`
  - QDRANT_API_KEY: `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.4m9T9I-NlGbJF6KZG0edJ4FS2xfOoYMCSlGYVbv-Mss`

---

## Deployment Option 1: Render.com (Recommended - Free Tier)

### Step 1: Prepare Your Git Repository

1. **Check if Step-2-rag-chatbot is a Git repository:**
   ```bash
   cd Step-2-rag-chatbot
   git status
   ```

2. **If NOT a git repo, initialize it:**
   ```bash
   git init
   git add .
   git commit -m "Initial commit - RAG chatbot backend"
   ```

3. **Create a new GitHub repository:**
   - Go to https://github.com/new
   - Name it: `rag-chatbot-backend`
   - Choose: Public or Private
   - DON'T initialize with README
   - Click "Create repository"

4. **Push your code to GitHub:**
   ```bash
   # Replace YOUR_USERNAME with your GitHub username
   git remote add origin https://github.com/YOUR_USERNAME/rag-chatbot-backend.git
   git branch -M main
   git push -u origin main
   ```

### Step 2: Deploy to Render

1. **Go to Render Dashboard:**
   - Visit: https://dashboard.render.com
   - Sign up or log in

2. **Create New Web Service:**
   - Click "New +" button
   - Select "Web Service"

3. **Connect GitHub Repository:**
   - Click "Connect GitHub" (first time only)
   - Authorize Render to access your repositories
   - Find and select: `rag-chatbot-backend`

4. **Configure the Web Service:**

   **Basic Settings:**
   - Name: `rag-chatbot-backend` (or your preferred name)
   - Region: `Oregon (US West)` or closest to you
   - Branch: `main`
   - Root Directory: Leave empty
   - Runtime: `Python 3`
   - Build Command: `pip install --upgrade pip && pip install -r requirements.txt`
   - Start Command: `python server.py`

   **Plan:**
   - Select: `Free` (this is important!)

5. **Add Environment Variables:**

   Click "Advanced" and add these environment variables:

   | Key | Value |
   |-----|-------|
   | `GEMINI_API_KEY` | `AIzaSyAnTWntUl05EyTX_RSJsaPIxeIWd3fiUyw` |
   | `COHERE_API_KEY` | `0DRQcyTI98p3HRpjuQv8tvg4IcQtRVBeublwiHoe` |
   | `QDRANT_URL` | `https://6eb3cc7d-3f4e-46a5-ae7c-20d8d583c238.europe-west3-0.gcp.cloud.qdrant.io:6333` |
   | `QDRANT_API_KEY` | `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.4m9T9I-NlGbJF6KZG0edJ4FS2xfOoYMCSlGYVbv-Mss` |
   | `DATABASE_URL` | `sqlite:///./test.db` |
   | `TEXTBOOK_COLLECTION_NAME` | `textbook_content` |
   | `PYTHON_VERSION` | `3.11.9` |

   **Important Notes:**
   - Copy-paste the values exactly as shown above
   - Don't add quotes around the values
   - Render will keep these secret

6. **Create Web Service:**
   - Click "Create Web Service" button
   - Wait for deployment (5-10 minutes)

7. **Monitor Deployment:**
   - Watch the logs in real-time
   - You should see:
     ```
     Installing dependencies...
     Building...
     Starting server...
     INFO: Uvicorn running on http://0.0.0.0:8000
     ```

8. **Get Your Backend URL:**
   - Once deployed, you'll see: `https://your-app-name.onrender.com`
   - Copy this URL (you'll need it for the frontend)

### Step 3: Test Your Deployed Backend

1. **Test Health Endpoint:**
   ```bash
   curl https://your-app-name.onrender.com/health
   ```

   Expected response:
   ```json
   {"status":"healthy","message":"All services are running"}
   ```

2. **Test Chat Endpoint:**
   ```bash
   curl -X POST https://your-app-name.onrender.com/api/v1/chat/completions \
     -H "Content-Type: application/json" \
     -d '{"messages": [{"role": "user", "content": "What is ROS 2?"}]}'
   ```

3. **If you get timeout on first request:**
   - This is normal for Render Free tier (cold start)
   - Wait 1-2 minutes and try again
   - The service spins down after 15 minutes of inactivity

### Step 4: Update Frontend Configuration

1. **Update your Docusaurus .env file:**
   ```bash
   cd Step-1-physical-ai-humanoid-textbook/docusaurus
   ```

2. **Edit `.env` file:**
   ```env
   # Production Backend URL
   REACT_APP_API_URL=https://your-app-name.onrender.com/api/v1/chat/completions
   ```

3. **Or update `docusaurus.config.ts`:**
   ```typescript
   customFields: {
     apiUrl: process.env.REACT_APP_API_URL || 'https://your-app-name.onrender.com/api/v1/chat/completions',
   },
   ```

4. **Restart your frontend:**
   ```bash
   npm start
   ```

---

## Deployment Option 2: Vercel (Alternative)

### Step 1: Install Vercel CLI

```bash
npm install -g vercel
```

### Step 2: Login to Vercel

```bash
vercel login
```

### Step 3: Deploy

```bash
cd Step-2-rag-chatbot
vercel
```

Follow the prompts:
- Set up and deploy? `Y`
- Which scope? Select your account
- Link to existing project? `N`
- What's your project's name? `rag-chatbot-backend`
- In which directory is your code located? `./`
- Want to override the settings? `Y`
  - Build Command: `pip install -r requirements.txt`
  - Output Directory: Leave empty
  - Development Command: `python server.py`

### Step 4: Add Environment Variables

```bash
vercel env add GEMINI_API_KEY
vercel env add COHERE_API_KEY
vercel env add QDRANT_URL
vercel env add QDRANT_API_KEY
vercel env add DATABASE_URL
vercel env add TEXTBOOK_COLLECTION_NAME
```

Enter the values when prompted.

### Step 5: Redeploy with Environment Variables

```bash
vercel --prod
```

---

## Deployment Option 3: Railway (Another Alternative)

### Step 1: Create Railway Account

- Go to https://railway.app
- Sign up with GitHub

### Step 2: New Project

1. Click "New Project"
2. Select "Deploy from GitHub repo"
3. Choose your `rag-chatbot-backend` repository

### Step 3: Configure

1. In the project settings:
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `python server.py`

2. Add Environment Variables in the "Variables" tab:
   - `GEMINI_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `DATABASE_URL`
   - `TEXTBOOK_COLLECTION_NAME`

3. Click "Deploy"

### Step 4: Get Your URL

- Railway will provide: `https://your-app.railway.app`
- Use this as your backend URL

---

## Important Notes for All Deployments

### 1. Free Tier Limitations

**Render.com Free Tier:**
- ✅ Free forever
- ❌ Spins down after 15 minutes of inactivity
- ❌ Cold start takes 30-60 seconds
- ✅ 750 hours/month
- ✅ Enough for development/testing

**Vercel Free Tier:**
- ✅ Free forever
- ✅ No cold starts
- ❌ Limited to serverless functions (may need adaptation)
- ✅ Great for production

**Railway Free Tier:**
- ✅ $5 free credit/month
- ✅ No cold starts
- ❌ Charges after $5
- ✅ Very fast deployment

### 2. Environment Variables

**Never commit these to Git:**
- GEMINI_API_KEY
- COHERE_API_KEY
- QDRANT_API_KEY
- QDRANT_URL

**Always add them through the platform's dashboard.**

### 3. Cold Start Solutions (Render Free Tier)

To keep your Render service warm:

**Option A: Use a cron job service (free):**
- https://cron-job.org
- Schedule to ping your endpoint every 10 minutes:
  ```
  GET https://your-app-name.onrender.com/health
  ```

**Option B: Use Render Cron Jobs:**
- Create a cron job in Render
- Schedule: `*/10 * * * *` (every 10 minutes)
- Command: `curl https://your-app-name.onrender.com/health`

### 4. CORS Configuration

Your backend already has CORS configured in `src/main.py`:
```python
allow_origins=["*"]
```

This allows requests from any origin. For production, you may want to restrict this:
```python
allow_origins=[
    "https://your-frontend-domain.com",
    "http://localhost:3000"
]
```

### 5. Monitoring

**Render Dashboard:**
- Go to your service
- Click "Logs" to see real-time logs
- Click "Metrics" to see usage

**Check Service Status:**
```bash
curl https://your-app-name.onrender.com/health
```

---

## Troubleshooting

### Issue 1: Deployment Fails

**Check:**
1. All environment variables are set correctly
2. requirements.txt is in root directory
3. Python version matches (3.11.9)

**Solution:**
- Check build logs in Render dashboard
- Look for missing dependencies
- Verify all required packages are in requirements.txt

### Issue 2: Service Times Out

**Cause:** Cold start on free tier

**Solutions:**
1. Wait 60 seconds and try again
2. Implement keep-alive ping (see above)
3. Upgrade to paid tier ($7/month)

### Issue 3: 500 Internal Server Error

**Check:**
1. Environment variables are set
2. Qdrant database is populated
3. API keys are valid

**Debug:**
```bash
# Check logs in Render dashboard
# Look for Python errors
```

### Issue 4: CORS Errors

**Solution:**
- Already configured in code
- If issue persists, check that frontend URL is allowed
- Verify request includes correct headers

---

## Post-Deployment Checklist

After successful deployment:

- [ ] Backend is accessible at the provided URL
- [ ] `/health` endpoint returns healthy status
- [ ] `/api/v1/chat/completions` accepts POST requests
- [ ] Environment variables are set correctly
- [ ] Qdrant database is populated (run `populate_db.py`)
- [ ] Frontend is updated with new backend URL
- [ ] Test end-to-end: Frontend → Backend → Response
- [ ] Set up monitoring/keep-alive (optional)
- [ ] Document your backend URL for team

---

## Estimated Costs

| Platform | Free Tier | Paid Tier |
|----------|-----------|-----------|
| **Render** | Free forever (with limitations) | $7/month (no cold starts) |
| **Vercel** | Free forever | $20/month |
| **Railway** | $5 credit/month | Pay as you go |

**Recommendation for Students/Development:**
- Use **Render Free Tier** + keep-alive service
- Total cost: **$0/month**

**Recommendation for Production:**
- Use **Render Starter** ($7/month)
- Or **Vercel** if adapted for serverless

---

## Next Steps After Deployment

1. **Populate Qdrant Database:**
   ```bash
   # From your local machine
   cd Step-2-rag-chatbot
   python populate_db.py
   ```

2. **Test Deployed Backend:**
   ```bash
   curl https://your-app-name.onrender.com/api/v1/chat/completions \
     -H "Content-Type: application/json" \
     -d '{"messages":[{"role":"user","content":"What is ROS 2?"}]}'
   ```

3. **Deploy Frontend:**
   - Update frontend with backend URL
   - Deploy frontend to Vercel/Netlify
   - Test end-to-end integration

4. **Set Up Custom Domain (Optional):**
   - Go to Render dashboard
   - Click "Settings" → "Custom Domain"
   - Add your domain (e.g., `api.yourdomain.com`)
   - Update DNS records as instructed

---

## Summary

Your backend is now deployed! 🎉

**Backend URL Format:**
```
https://your-app-name.onrender.com
```

**API Endpoints:**
- Health: `GET /health`
- Chat: `POST /api/v1/chat/completions`
- Root: `GET /`

**Features:**
- ✅ Gemini API for generation
- ✅ Cohere for embeddings
- ✅ Qdrant for vector search
- ✅ CORS enabled
- ✅ Environment variables secured

**No code changes needed!** Everything is ready to deploy as-is.
