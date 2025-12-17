/**
 * Protected Route Component
 * Wraps components that require authentication
 */

import React, { useEffect, useState, ReactNode } from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import { getAuthToken } from '@site/src/utils/auth';

interface ProtectedRouteProps {
    children: ReactNode;
}

const ProtectedRoute: React.FC<ProtectedRouteProps> = ({ children }) => {
    const { isAuthenticated, verifyToken, loading } = useAuth();
    const [isVerifying, setIsVerifying] = useState(true);
    const [shouldRender, setShouldRender] = useState(false);

    // Immediate check for token - don't wait for context
    useEffect(() => {
        const token = getAuthToken();
        if (!token) {
            console.log('No token found, redirecting to login...');
            window.location.href = '/login';
            return;
        }
    }, []);

    useEffect(() => {
        const checkAuth = async () => {
            if (loading) {
                return;
            }

            if (!isAuthenticated) {
                console.log('Not authenticated, redirecting to login...');
                window.location.href = '/login';
                return;
            }

            // Verify token with backend
            console.log('Verifying token with backend...');
            const isValid = await verifyToken();
            if (!isValid) {
                console.log('Token invalid, redirecting to login...');
                window.location.href = '/login';
                return;
            }

            console.log('Token valid, showing chatbot...');
            setIsVerifying(false);
            setShouldRender(true);
        };

        checkAuth();
    }, [isAuthenticated, loading, verifyToken]);

    // Show loading state while verifying
    if (loading || isVerifying || !shouldRender) {
        return (
            <div style={{
                display: 'flex',
                justifyContent: 'center',
                alignItems: 'center',
                height: '100vh',
                fontSize: '18px',
                color: '#666'
            }}>
                Checking authentication...
            </div>
        );
    }

    // Only render children if authenticated
    return isAuthenticated ? <>{children}</> : null;
};

export default ProtectedRoute;
