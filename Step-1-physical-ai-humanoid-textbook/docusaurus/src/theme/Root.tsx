import React from 'react';
import Chatbot from '@site/src/components/Chatbot';

// This component wraps the entire app and is loaded on every page
export default function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}
