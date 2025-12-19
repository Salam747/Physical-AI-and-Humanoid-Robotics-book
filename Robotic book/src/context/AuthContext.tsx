/**
 * Authentication Context Provider
 * Manages user authentication state across the application
 */

import React, { createContext, useState, useEffect, useContext, ReactNode } from 'react';
import {
    User,
    AuthResponse,
    API_URL,
    getAuthToken,
    setAuthToken as saveAuthToken,
    removeAuthToken,
    isTokenExpired,
    decodeToken,
    authenticatedFetch
} from '@site/src/utils/auth';

interface SignupData {
    first_name: string;
    last_name: string;
    email: string;
    password: string;
    confirm_password: string;
}

interface AuthContextType {
    user: User | null;
    token: string | null;
    isAuthenticated: boolean;
    loading: boolean;
    login: (email: string, password: string) => Promise<void>;
    signup: (data: SignupData) => Promise<void>;
    logout: () => void;
    verifyToken: () => Promise<boolean>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: ReactNode }> = ({ children }) => {
    const [user, setUser] = useState<User | null>(null);
    const [token, setToken] = useState<string | null>(null);
    const [loading, setLoading] = useState<boolean>(true);

    // Initialize auth state from localStorage
    useEffect(() => {
        const initAuth = async () => {
            const storedToken = getAuthToken();

            if (storedToken && !isTokenExpired(storedToken)) {
                // Immediately set user from token (optimistic update)
                const userData = decodeToken(storedToken);
                setToken(storedToken);
                setUser({
                    id: parseInt(userData.sub),
                    first_name: userData.first_name,
                    last_name: userData.last_name,
                    email: userData.email
                });
                setLoading(false);

                // Verify in background
                verifyTokenWithBackend(storedToken).then((isValid) => {
                    if (!isValid) {
                        // If verification fails, clear everything
                        removeAuthToken();
                        setToken(null);
                        setUser(null);
                    }
                });
            } else {
                setLoading(false);
            }
        };

        initAuth();
    }, []);

    /**
     * Verify token with backend
     */
    const verifyTokenWithBackend = async (tokenToVerify: string): Promise<boolean> => {
        try {
            const response = await fetch(`${API_URL}/auth/verify`, {
                method: 'GET',
                headers: {
                    'Authorization': `Bearer ${tokenToVerify}`
                }
            });

            if (response.ok) {
                return true;
            }
            return false;
        } catch (error) {
            console.error('Token verification failed:', error);
            return false;
        }
    };

    /**
     * Verify current token
     */
    const verifyToken = async (): Promise<boolean> => {
        const currentToken = getAuthToken();
        if (!currentToken) return false;

        if (isTokenExpired(currentToken)) {
            logout();
            return false;
        }

        return await verifyTokenWithBackend(currentToken);
    };

    /**
     * Sign up new user
     */
    const signup = async (data: SignupData): Promise<void> => {
        try {
            const response = await fetch(`${API_URL}/auth/signup`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(data)
            });

            if (!response.ok) {
                const errorData = await response.json();
                throw new Error(errorData.detail || 'Signup failed');
            }

            const result: AuthResponse = await response.json();

            if (result.success && result.token && result.user) {
                saveAuthToken(result.token);
                setToken(result.token);
                setUser(result.user);
                // Don't redirect here - let the form component handle it
                // Yahan redirect mat karo - form component handle karega
            } else {
                throw new Error(result.message || 'Signup failed');
            }
        } catch (error) {
            throw error;
        }
    };

    /**
     * Log in existing user
     */
    const login = async (email: string, password: string): Promise<void> => {
        try {
            const response = await fetch(`${API_URL}/auth/login`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify({ email, password })
            });

            if (!response.ok) {
                const errorData = await response.json();
                throw new Error(errorData.detail || 'Login failed');
            }

            const result: AuthResponse = await response.json();

            if (result.success && result.token && result.user) {
                saveAuthToken(result.token);
                setToken(result.token);
                setUser(result.user);
                // Don't redirect here - let the form component handle it
                // Yahan redirect mat karo - form component handle karega
            } else {
                throw new Error(result.message || 'Login failed');
            }
        } catch (error) {
            throw error;
        }
    };

    /**
     * Log out user
     */
    const logout = (): void => {
        removeAuthToken();
        setToken(null);
        setUser(null);
        window.location.href = '/';
    };

    const value: AuthContextType = {
        user,
        token,
        isAuthenticated: !!user && !!token,
        loading,
        login,
        signup,
        logout,
        verifyToken
    };

    return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

/**
 * Hook to use auth context
 */
export const useAuth = (): AuthContextType => {
    const context = useContext(AuthContext);
    if (context === undefined) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
};
