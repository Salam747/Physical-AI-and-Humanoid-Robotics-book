/**
 * Chatbot Page
 * Protected page for AI chatbot with authentication
 */

import React, { useEffect, useState } from 'react';
import Layout from '@theme/Layout';
import Chatbot from '@site/src/components/Chatbot';
import { useAuth } from '@site/src/context/AuthContext';
import { getAuthToken } from '@site/src/utils/auth';
import styles from './chatbot.module.css';

export default function ChatbotPage(): JSX.Element {
    const { user, logout, isAuthenticated, loading } = useAuth();
    const [isChecking, setIsChecking] = useState(true);

    useEffect(() => {
        // Immediate redirect if no token
        const token = getAuthToken();
        if (!token) {
            window.location.href = '/login';
            return;
        }

        // Wait for auth context to load
        if (!loading) {
            if (!isAuthenticated) {
                window.location.href = '/login';
            } else {
                setIsChecking(false);
            }
        }
    }, [loading, isAuthenticated]);

    if (loading || isChecking) {
        return (
            <Layout title="AI Chatbot">
                <div style={{
                    display: 'flex',
                    justifyContent: 'center',
                    alignItems: 'center',
                    height: '80vh',
                    fontSize: '18px',
                    color: '#666'
                }}>
                    Checking authentication...
                </div>
            </Layout>
        );
    }

    if (!isAuthenticated) {
        return null;
    }

    return (
        <Layout
            title="AI Chatbot"
            description="Ask questions about Physical AI and Humanoid Robotics">
            <div className={styles.chatbotPageContainer}>
                <div className={styles.chatbotHeader}>
                    <div className={styles.headerContent}>
                        <h1>AI Chatbot</h1>
                        <p>Ask me anything about Physical AI and Humanoid Robotics!</p>
                    </div>
                    <div className={styles.userSection}>
                        <div className={styles.userInfo}>
                            <span className={styles.userAvatar}>
                                {user?.first_name?.charAt(0).toUpperCase()}
                                {user?.last_name?.charAt(0).toUpperCase()}
                            </span>
                            <div className={styles.userName}>
                                <strong>{user?.first_name} {user?.last_name}</strong>
                                <small>{user?.email}</small>
                            </div>
                        </div>
                        <button
                            onClick={logout}
                            className={styles.logoutButton}>
                            Logout
                        </button>
                    </div>
                </div>

                <div className={styles.chatbotContent}>
                    <Chatbot />
                </div>
            </div>
        </Layout>
    );
}
