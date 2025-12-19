import React, { useState, useEffect, useRef } from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
import { getAuthToken, API_URL } from '@site/src/utils/auth';
import AuthModal from '@site/src/components/Auth/AuthModal';

// Check if user is authenticated
const isUserAuthenticated = () => {
  const token = getAuthToken();
  return !!token;
};

// Premium Robot Icon SVG
const RobotIcon = () => (
  <svg width="32" height="32" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 2C12.5523 2 13 2.44772 13 3V4H14C14.5523 4 15 4.44772 15 5C15 5.55228 14.5523 6 14 6H13V7H16C17.6569 7 19 8.34315 19 10V11H20C20.5523 11 21 11.4477 21 12C21 12.5523 20.5523 13 20 13H19V17C19 18.6569 17.6569 20 16 20H8C6.34315 20 5 18.6569 5 17V13H4C3.44772 13 3 12.5523 3 12C3 11.4477 3.44772 11 4 11H5V10C5 8.34315 6.34315 7 8 7H11V6H10C9.44772 6 9 5.55228 9 5C9 4.44772 9.44772 4 10 4H11V3C11 2.44772 11.4477 2 12 2Z" fill="currentColor"/>
    <circle cx="9" cy="12" r="1.5" fill="white"/>
    <circle cx="15" cy="12" r="1.5" fill="white"/>
    <path d="M9 16H15" stroke="white" strokeWidth="1.5" strokeLinecap="round"/>
    <path d="M2 10L4 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
    <path d="M2 14L4 13" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
    <path d="M22 10L20 11" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
    <path d="M22 14L20 13" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round"/>
  </svg>
);

// Small Robot Icon for Header
const SmallRobotIcon = () => (
  <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M12 2C12.5523 2 13 2.44772 13 3V4H14C14.5523 4 15 4.44772 15 5C15 5.55228 14.5523 6 14 6H13V7H16C17.6569 7 19 8.34315 19 10V11H20C20.5523 11 21 11.4477 21 12C21 12.5523 20.5523 13 20 13H19V17C19 18.6569 17.6569 20 16 20H8C6.34315 20 5 18.6569 5 17V13H4C3.44772 13 3 12.5523 3 12C3 11.4477 3.44772 11 4 11H5V10C5 8.34315 6.34315 7 8 7H11V6H10C9.44772 6 9 5.55228 9 5C9 4.44772 9.44772 4 10 4H11V3C11 2.44772 11.4477 2 12 2Z" fill="currentColor"/>
    <circle cx="9" cy="12" r="1.5" fill="white"/>
    <circle cx="15" cy="12" r="1.5" fill="white"/>
    <path d="M9 16H15" stroke="white" strokeWidth="1.5" strokeLinecap="round"/>
  </svg>
);

// Send Icon
const SendIcon = () => (
  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
    <path d="M22 2L11 13" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
    <path d="M22 2L15 22L11 13L2 9L22 2Z" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
  </svg>
);

function Chatbot() {
  const [isOpen, setIsOpen] = useState(false);
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    // Check authentication before opening chatbot
    if (!isUserAuthenticated()) {
      // Show auth modal instead of redirecting
      setShowAuthModal(true);
      return;
    }
    setIsOpen(!isOpen);
  };

  const handleAuthSuccess = () => {
    // Close modal and open chatbot after successful auth
    setShowAuthModal(false);
    setIsOpen(true);
  };

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    const userMessage = { sender: 'user', text: inputValue };
    const currentQuestion = inputValue.trim().toLowerCase();
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInputValue('');
    setIsLoading(true);

    // Check if it's a simple greeting - respond instantly without API call
    const greetings = ['hello', 'hi', 'hey', 'good morning', 'good afternoon', 'good evening', 'greetings'];
    if (greetings.includes(currentQuestion)) {
      setIsLoading(false);
      const greetingResponse = {
        sender: 'bot',
        text: "Hello! I'm your AI assistant for Physical AI & Humanoid Robotics. I can help you understand concepts about ROS 2, Digital Twins, Reinforcement Learning, and Vision-Language-Action models. How can I help you today?"
      };
      setMessages((prevMessages) => [...prevMessages, greetingResponse]);
      return;
    }

    try {
      const token = getAuthToken();
      const headers = {
        'Content-Type': 'application/json',
      };

      // Add Authorization header if token exists
      if (token) {
        headers['Authorization'] = `Bearer ${token}`;
      }

      const response = await fetch(`${API_URL}/query`, {
        method: 'POST',
        headers: headers,
        body: JSON.stringify({ question: currentQuestion }),
      });

      if (!response.ok) {
        if (response.status === 401) {
          throw new Error('Session expired. Please login again.');
        } else if (response.status === 500) {
          throw new Error('Server error. Please ensure the backend is running.');
        }
        throw new Error('Network response was not ok');
      }

      const data = await response.json();
      const botMessage = {
        sender: 'bot',
        text: data.answer || 'I apologize, but I couldn\'t generate a response. Please try rephrasing your question.',
        sources: data.source_chunks || []
      };
      setMessages((prevMessages) => [...prevMessages, botMessage]);
    } catch (error) {
      console.error('Error fetching data:', error);
      let errorText = 'Sorry, something went wrong. ';

      if (error.message.includes('Session expired')) {
        errorText = 'Your session has expired. Please close and reopen the chatbot to login again.';
      } else if (error.message.includes('Server error')) {
        errorText = 'Unable to connect to the backend server. Please ensure the server is running at http://127.0.0.1:8000';
      } else if (error.message.includes('Failed to fetch')) {
        errorText = 'Network error. Please check your internet connection and ensure the backend is running.';
      } else {
        errorText += 'Please try again.';
      }

      const errorMessage = { sender: 'bot', text: errorText };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Authentication Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={handleAuthSuccess}
        defaultTab="login"
      />

      {/* Floating Chat Button */}
      <div className={styles.chatbotIcon} onClick={toggleChat}>
        <RobotIcon />
      </div>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div className={styles.headerContent}>
              <div className={styles.headerIcon}>
                <SmallRobotIcon />
              </div>
              <div className={styles.headerText}>
                <span className={styles.headerTitle}>AI Assistant</span>
                <span className={styles.headerSubtitle}>
                  <span className={styles.onlineDot}></span>
                  Physical AI & Robotics
                </span>
              </div>
            </div>
            <button onClick={toggleChat} className={styles.closeButton}>✕</button>
          </div>

          {/* Messages */}
          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={clsx(styles.message, styles.bot)}>
                <div className={styles.welcomeHeader}>
                  <span className={styles.botIcon}>🤖</span>
                  <strong>AI Assistant</strong>
                </div>
                <p className={styles.welcomeText}>
                  Hello! I specialize in Physical AI & Humanoid Robotics. Ask me about ROS 2, Digital Twins, Reinforcement Learning, or Vision-Language-Action models.
                </p>
                <p className={styles.welcomePrompt}>How can I help you today?</p>
              </div>
            )}
            {messages.map((message, index) => (
              <div key={index} className={clsx(styles.message, styles[message.sender])}>
                {message.text}
              </div>
            ))}
            {isLoading && (
              <div className={clsx(styles.message, styles.bot)}>
                <div className={styles.loadingContainer}>
                  <div className={styles.loadingDots}>
                    <span></span><span></span><span></span>
                  </div>
                  <span className={styles.loadingText}>Searching knowledge base...</span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              placeholder="Ask me anything..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button type="submit" className={styles.sendButton} disabled={isLoading}>
              <SendIcon />
            </button>
          </form>
        </div>
      )}
    </>
  );
}

export default Chatbot;
