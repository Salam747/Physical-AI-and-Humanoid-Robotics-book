/**
 * AuthModal Component
 * Modal overlay for authentication with login/signup tabs
 * Opens on top of current page with blur background
 */

import React, { useState, useEffect } from 'react';
import SignUpForm from './SignUpForm';
import LoginForm from './LoginForm';
import styles from './AuthModal.module.css';

interface AuthModalProps {
    isOpen: boolean;
    onClose: () => void;
    onSuccess?: () => void;
    defaultTab?: 'login' | 'signup';
}

const AuthModal: React.FC<AuthModalProps> = ({
    isOpen,
    onClose,
    onSuccess,
    defaultTab = 'login'
}) => {
    const [activeTab, setActiveTab] = useState<'login' | 'signup'>(defaultTab);

    // Reset tab when modal opens
    useEffect(() => {
        if (isOpen) {
            setActiveTab(defaultTab);
        }
    }, [isOpen, defaultTab]);

    // Close on Escape key
    useEffect(() => {
        const handleEscape = (e: KeyboardEvent) => {
            if (e.key === 'Escape' && isOpen) {
                onClose();
            }
        };
        window.addEventListener('keydown', handleEscape);
        return () => window.removeEventListener('keydown', handleEscape);
    }, [isOpen, onClose]);

    // Prevent body scroll when modal is open
    useEffect(() => {
        if (isOpen) {
            document.body.style.overflow = 'hidden';
        } else {
            document.body.style.overflow = 'unset';
        }
        return () => {
            document.body.style.overflow = 'unset';
        };
    }, [isOpen]);

    if (!isOpen) return null;

    return (
        <div className={styles.modalOverlay} onClick={onClose}>
            <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
                {/* Close Button */}
                <button
                    className={styles.closeButton}
                    onClick={onClose}
                    aria-label="Close"
                >
                    ✕
                </button>

                {/* Modal Header */}
                <div className={styles.modalHeader}>
                    <div className={styles.robotIcon}>
                        <svg width="48" height="48" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
                            <path d="M12 2C12.5523 2 13 2.44772 13 3V4H14C14.5523 4 15 4.44772 15 5C15 5.55228 14.5523 6 14 6H13V7H16C17.6569 7 19 8.34315 19 10V11H20C20.5523 11 21 11.4477 21 12C21 12.5523 20.5523 13 20 13H19V17C19 18.6569 17.6569 20 16 20H8C6.34315 20 5 18.6569 5 17V13H4C3.44772 13 3 12.5523 3 12C3 11.4477 3.44772 11 4 11H5V10C5 8.34315 6.34315 7 8 7H11V6H10C9.44772 6 9 5.55228 9 5C9 4.44772 9.44772 4 10 4H11V3C11 2.44772 11.4477 2 12 2Z" fill="currentColor"/>
                            <circle cx="9" cy="12" r="1.5" fill="white"/>
                            <circle cx="15" cy="12" r="1.5" fill="white"/>
                            <path d="M9 16H15" stroke="white" strokeWidth="1.5" strokeLinecap="round"/>
                        </svg>
                    </div>
                    <h2>Welcome to AI Chatbot</h2>
                    <p>Ask questions about Physical AI and Humanoid Robotics</p>
                </div>

                {/* Tabs */}
                <div className={styles.tabs}>
                    <button
                        className={`${styles.tab} ${activeTab === 'login' ? styles.active : ''}`}
                        onClick={() => setActiveTab('login')}
                    >
                        Log In
                    </button>
                    <button
                        className={`${styles.tab} ${activeTab === 'signup' ? styles.active : ''}`}
                        onClick={() => setActiveTab('signup')}
                    >
                        Sign Up
                    </button>
                </div>

                {/* Form Content */}
                <div className={styles.formContent}>
                    {activeTab === 'login' ? (
                        <LoginForm onSuccess={onSuccess} />
                    ) : (
                        <SignUpForm onSuccess={onSuccess} />
                    )}
                </div>

                {/* Footer */}
                <div className={styles.modalFooter}>
                    {activeTab === 'login' ? (
                        <p>
                            Don't have an account?{' '}
                            <button
                                className={styles.switchButton}
                                onClick={() => setActiveTab('signup')}
                            >
                                Sign up here
                            </button>
                        </p>
                    ) : (
                        <p>
                            Already have an account?{' '}
                            <button
                                className={styles.switchButton}
                                onClick={() => setActiveTab('login')}
                            >
                                Log in here
                            </button>
                        </p>
                    )}
                </div>
            </div>
        </div>
    );
};

export default AuthModal;
