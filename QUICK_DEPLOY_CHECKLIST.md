# Quick Deployment Checklist

## 🚀 Deploy Backend to Render in 10 Minutes

### Pre-Deployment (5 minutes)

#### 1. Commit Your Changes
```bash
cd Step-2-rag-chatbot
git add .
git commit -m "Update to Gemini API with gRPC fix"
git push origin main
```

#### 2. Have These Ready
- [ ] GitHub account logged in
- [ ] Render.com account (create at https://render.com)
- [ ] These API keys copied:
  ```
  GEMINI_API_KEY=AIzaSyAnTWntUl05EyTX_RSJsaPIxeIWd3fiUyw
  COHERE_API_KEY=0DRQcyTI98p3HRpjuQv8tvg4IcQtRVBeublwiHoe
  QDRANT_URL=https://6eb3cc7d-3f4e-46a5-ae7c-20d8d583c238.europe-west3-0.gcp.cloud.qdrant.io:6333
  QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.4m9T9I-NlGbJF6KZG0edJ4FS2xfOoYMCSlGYVbv-Mss
  ```

---

### Deployment Steps (5 minutes)

#### Step 1: Go to Render
1. Visit: https://dashboard.render.com
2. Click "New +" → "Web Service"

#### Step 2: Connect Repository
1. Click "Connect GitHub"
2. Select your repository (the one with Step-2-rag-chatbot)

#### Step 3: Configure Settings
**Fill in these fields:**

| Field | Value |
|-------|-------|
| Name | `rag-chatbot-backend` |
| Region | `Oregon (US West)` |
| Branch | `main` |
| Root Directory | `Step-2-rag-chatbot` |
| Runtime | `Python 3` |
| Build Command | `pip install --upgrade pip && pip install -r requirements.txt` |
| Start Command | `python server.py` |
| Plan | **Free** |

#### Step 4: Add Environment Variables
Click "Advanced" → "Add Environment Variable"

Add these 6 variables:

1. `GEMINI_API_KEY` = `AIzaSyAnTWntUl05EyTX_RSJsaPIxeIWd3fiUyw`
2. `COHERE_API_KEY` = `0DRQcyTI98p3HRpjuQv8tvg4IcQtRVBeublwiHoe`
3. `QDRANT_URL` = `https://6eb3cc7d-3f4e-46a5-ae7c-20d8d583c238.europe-west3-0.gcp.cloud.qdrant.io:6333`
4. `QDRANT_API_KEY` = `eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.4m9T9I-NlGbJF6KZG0edJ4FS2xfOoYMCSlGYVbv-Mss`
5. `DATABASE_URL` = `sqlite:///./test.db`
6. `TEXTBOOK_COLLECTION_NAME` = `textbook_content`

#### Step 5: Deploy
1. Click "Create Web Service"
2. Wait 5-10 minutes for build
3. Copy your URL: `https://rag-chatbot-backend-xxx.onrender.com`

---

### Post-Deployment Testing (2 minutes)

#### Test 1: Health Check
```bash
curl https://YOUR-URL.onrender.com/health
```

**Expected:** `{"status":"healthy","message":"All services are running"}`

⚠️ **First request might take 60 seconds (cold start)**

#### Test 2: Chat Completion
```bash
curl -X POST https://YOUR-URL.onrender.com/api/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"messages":[{"role":"user","content":"What is ROS 2?"}]}'
```

**Expected:** JSON response with answer

---

### Update Frontend (1 minute)

#### Option A: Edit .env file
```bash
cd Step-1-physical-ai-humanoid-textbook/docusaurus
```

Edit `.env`:
```env
REACT_APP_API_URL=https://YOUR-URL.onrender.com/api/v1/chat/completions
```

#### Option B: Edit docusaurus.config.ts
Change line 25:
```typescript
apiUrl: process.env.REACT_APP_API_URL || 'https://YOUR-URL.onrender.com/api/v1/chat/completions',
```

#### Restart Frontend
```bash
npm start
```

---

## ✅ Deployment Complete!

Your backend is now live at:
```
https://YOUR-URL.onrender.com
```

### Test End-to-End:
1. Open http://localhost:3000
2. Click chatbot button
3. Ask "What is ROS 2?"
4. Wait ~10 seconds for response

---

## 🔧 Troubleshooting

### Issue: Timeout on First Request
- **Normal** for free tier
- Wait 60 seconds
- Try again

### Issue: 500 Error
1. Check Render logs
2. Verify all 6 environment variables are set
3. Check API keys are correct

### Issue: Build Fails
1. Check `requirements.txt` is in root
2. Check Python version is 3.11
3. Check Render logs for errors

---

## 💡 Pro Tips

### Keep Service Warm (No Cold Starts)
Use free cron service: https://cron-job.org

**Setup:**
1. Create account
2. Add job:
   - URL: `https://YOUR-URL.onrender.com/health`
   - Schedule: Every 10 minutes
   - Method: GET

### Monitor Your Service
**Render Dashboard:**
- View logs in real-time
- Check metrics
- See deploy history

**URL:** https://dashboard.render.com/web/YOUR-SERVICE-ID

---

## 📊 What You Get

### Free Tier Includes:
- ✅ 750 hours/month (enough for 24/7)
- ✅ Automatic HTTPS
- ✅ Automatic deployments on push
- ✅ Environment variables
- ✅ Logs and metrics

### Limitations:
- ⏱️ Spins down after 15 min inactivity
- ⏱️ Cold start: 30-60 seconds
- 💾 Limited resources

### Upgrade ($7/month):
- ⚡ No cold starts
- ⚡ Faster resources
- ⚡ Priority support

---

## 🎯 Next Steps

1. ✅ Backend deployed
2. ⬜ Test thoroughly
3. ⬜ Update frontend URL
4. ⬜ Deploy frontend to Vercel
5. ⬜ Set up custom domain (optional)
6. ⬜ Enable keep-alive (optional)

---

## 📝 Important URLs

- **Your Backend**: `https://YOUR-URL.onrender.com`
- **Render Dashboard**: https://dashboard.render.com
- **Documentation**: See `DEPLOYMENT_GUIDE.md`

---

## 🆘 Need Help?

1. Check `DEPLOYMENT_GUIDE.md` for detailed instructions
2. Check Render logs for errors
3. Test locally first: `python server.py`
4. Verify environment variables in Render dashboard

---

**Deployment Time: ~10 minutes**
**Cost: $0/month (Free tier)**
**Uptime: ~98% (with keep-alive)**

Good luck! 🚀
