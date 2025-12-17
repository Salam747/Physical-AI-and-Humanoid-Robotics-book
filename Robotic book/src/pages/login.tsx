/**
 * Login Page
 * Authentication page with Sign Up and Log In tabs
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/context/AuthContext';
import SignUpForm from '@site/src/components/Auth/SignUpForm';
import LoginForm from '@site/src/components/Auth/LoginForm';
import styles from './login.module.css';

export default function LoginPage(): JSX.Element {
    const [activeTab, setActiveTab] = useState<'login' | 'signup'>('login');
    const { isAuthenticated, loading } = useAuth();

    // Redirect to chatbot if already authenticated
    useEffect(() => {
        if (!loading && isAuthenticated) {
            window.location.href = '/chatbot';
        }
    }, [isAuthenticated, loading]);

    if (loading) {
        return (
            <Layout title="Login">
                <div className={styles.loadingContainer}>
                    <div className={styles.spinner}></div>
                    <p>Loading...</p>
                </div>
            </Layout>
        );
    }

    return (
        <Layout
            title="Login"
            description="Sign in to access the AI Chatbot for Physical AI and Humanoid Robotics">
            <div className={styles.authContainer}>
                <div className={styles.authCard}>
                    <div className={styles.authHeader}>
                        <h1>Welcome to AI Chatbot</h1>
                        <p>Sign in to ask questions about Physical AI and Humanoid Robotics</p>
                    </div>

                    <div className={styles.tabs}>
                        <button
                            className={`${styles.tab} ${activeTab === 'login' ? styles.active : ''}`}
                            onClick={() => setActiveTab('login')}>
                            Log In
                        </button>
                        <button
                            className={`${styles.tab} ${activeTab === 'signup' ? styles.active : ''}`}
                            onClick={() => setActiveTab('signup')}>
                            Sign Up
                        </button>
                    </div>

                    <div className={styles.tabContent}>
                        {activeTab === 'login' ? <LoginForm /> : <SignUpForm />}
                    </div>

                    <div className={styles.authFooter}>
                        {activeTab === 'login' ? (
                            <p>
                                Don't have an account?{' '}
                                <button
                                    className={styles.switchButton}
                                    onClick={() => setActiveTab('signup')}>
                                    Sign up here
                                </button>
                            </p>
                        ) : (
                            <p>
                                Already have an account?{' '}
                                <button
                                    className={styles.switchButton}
                                    onClick={() => setActiveTab('login')}>
                                    Log in here
                                </button>
                            </p>
                        )}
                    </div>
                </div>
            </div>
        </Layout>
    );
}
