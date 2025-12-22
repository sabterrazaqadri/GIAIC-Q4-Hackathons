# ChatKit Chatbot Integration Guide

This guide explains how the AI chatbot is integrated into the Docusaurus textbook website.

## Overview

The chatbot is a floating chat widget that appears on all pages of the Docusaurus site. It connects to your backend API to answer questions about the Physical AI & Humanoid Robotics textbook content.

## Architecture

### Components

1. **Chatbot Component** (`src/components/Chatbot/Chatbot.tsx`)
   - Main React component with chat UI
   - Handles message state and API communication
   - Responsive design with dark mode support

2. **Chatbot Styles** (`src/components/Chatbot/Chatbot.module.css`)
   - CSS modules for styling
   - Gradient theme matching the textbook branding
   - Mobile-responsive layout

3. **Root Wrapper** (`src/theme/Root.tsx`)
   - Docusaurus theme component
   - Wraps the entire app to include chatbot on all pages

## Setup Instructions

### 1. Environment Configuration

Create a `.env` file in the `docusaurus/` directory (copy from `.env.example`):

```bash
cp .env.example .env
```

Edit the `.env` file to set your backend API URL:

```env
REACT_APP_API_URL=http://localhost:8001/api/v1/chat/completions
```

For production, update this to your deployed backend URL.

### 2. Start the Backend Server

Make sure your FastAPI backend is running:

```bash
cd Backend
python main.py
```

The backend should be running on `http://localhost:8001`

### 3. Start Docusaurus Development Server

```bash
cd docusaurus
npm start
```

The site will open at `http://localhost:3000`

### 4. Test the Chatbot

1. Look for the purple chat button (💬) in the bottom-right corner
2. Click it to open the chat window
3. Type a question about the textbook content
4. The chatbot will retrieve relevant information and respond

## Features

### User Interface
- **Floating Button**: Always accessible in bottom-right corner
- **Chat Window**: Clean, modern design with smooth animations
- **Message Bubbles**: Distinct styling for user and bot messages
- **Typing Indicator**: Shows when the bot is processing
- **Timestamps**: Each message shows the time it was sent
- **Dark Mode**: Automatically adapts to Docusaurus theme

### Functionality
- **Real-time Chat**: Instant communication with the AI backend
- **Context-Aware**: Retrieves relevant textbook content using RAG
- **Error Handling**: Graceful error messages if backend is unavailable
- **Keyboard Support**: Press Enter to send messages
- **Auto-scroll**: Automatically scrolls to latest message

## API Integration

The chatbot communicates with your backend API at:
```
POST /api/v1/chat/completions
```

**Request Format:**
```json
{
  "messages": [
    { "role": "user", "content": "What is URDF?" }
  ],
  "temperature": 0.7
}
```

**Response Format:**
```json
{
  "choices": [
    {
      "message": {
        "role": "assistant",
        "content": "URDF stands for..."
      }
    }
  ]
}
```

## Customization

### Changing Colors

Edit `Chatbot.module.css` to change the color scheme:

```css
/* Change gradient colors */
background: linear-gradient(135deg, #YOUR_COLOR_1 0%, #YOUR_COLOR_2 100%);
```

### Modifying Behavior

Edit `Chatbot.tsx` to customize:
- Initial greeting message (line 10)
- API endpoint (line 46)
- Request parameters (line 55)
- Temperature setting (line 57)

### Adjusting Position

Change the button position in `Chatbot.module.css`:

```css
.chatButton {
  bottom: 20px;  /* Distance from bottom */
  right: 20px;   /* Distance from right */
}
```

## Troubleshooting

### Chatbot Not Appearing
- Check that `src/theme/Root.tsx` exists
- Restart the Docusaurus dev server
- Clear browser cache

### API Connection Errors
- Verify backend is running: `curl http://localhost:8001/health`
- Check CORS settings in backend `main.py`
- Verify API URL in `.env` file
- Check browser console for error messages

### CORS Issues
Make sure your backend has CORS configured:

```python
# In Backend/src/api/main.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

### Styling Issues
- Clear Docusaurus cache: `npm run clear`
- Check CSS module imports
- Verify dark mode selectors: `[data-theme='dark']`

## Production Deployment

### 1. Update Environment Variables

For production builds, set the production API URL:

```env
REACT_APP_API_URL=https://your-backend-domain.com/api/v1/chat/completions
```

### 2. Build Docusaurus

```bash
npm run build
```

### 3. Deploy

Deploy the `build/` directory to your hosting service (Vercel, Netlify, etc.)

### 4. CORS Configuration

Update backend CORS to allow your production domain:

```python
allow_origins=[
    "http://localhost:3000",
    "https://your-docusaurus-domain.com"
]
```

## Advanced Configuration

### Adding Authentication

To add user authentication:

1. Modify `Chatbot.tsx` to include auth headers:
```typescript
headers: {
  'Content-Type': 'application/json',
  'Authorization': `Bearer ${authToken}`,
}
```

2. Update backend to validate tokens

### Session Management

To maintain conversation history:

1. Store session ID in component state
2. Include session ID in API requests
3. Backend should retrieve conversation history

### Analytics

Track chatbot usage:

```typescript
// In sendMessage function
analytics.track('chatbot_message_sent', {
  question: input,
  timestamp: new Date(),
});
```

## Support

For issues or questions:
- Check backend logs: `Backend/logs/`
- Review browser console for errors
- Verify API responses with curl or Postman

## License

This chatbot integration is part of the GIAIC Physical AI & Humanoid Robotics textbook project.
