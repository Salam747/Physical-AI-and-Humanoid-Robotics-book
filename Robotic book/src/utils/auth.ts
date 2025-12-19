/**
 * Authentication utility functions
 * Token management and API configuration
 */

import { jwtDecode } from 'jwt-decode';

// API URL configuration
export const API_URL = process.env.NODE_ENV === 'production'
    ? 'https://your-vercel-backend.vercel.app'  // Update this with your Vercel URL
    : 'http://127.0.0.1:8000';

// Local storage key for auth token
const AUTH_TOKEN_KEY = 'auth_token';

/**
 * Save JWT token to localStorage
 */
export const setAuthToken = (token: string): void => {
    localStorage.setItem(AUTH_TOKEN_KEY, token);
};

/**
 * Get JWT token from localStorage
 */
export const getAuthToken = (): string | null => {
    return localStorage.getItem(AUTH_TOKEN_KEY);
};

/**
 * Remove JWT token from localStorage
 */
export const removeAuthToken = (): void => {
    localStorage.removeItem(AUTH_TOKEN_KEY);
};

/**
 * Check if JWT token is expired
 */
export const isTokenExpired = (token: string): boolean => {
    try {
        const decoded: any = jwtDecode(token);
        if (!decoded.exp) return true;

        // Token expiration is in seconds, Date.now() is in milliseconds
        return decoded.exp * 1000 < Date.now();
    } catch (error) {
        return true;
    }
};

/**
 * Decode user data from JWT token
 */
export const decodeToken = (token: string): any => {
    try {
        return jwtDecode(token);
    } catch (error) {
        return null;
    }
};

/**
 * API request helper with authentication
 */
export const authenticatedFetch = async (
    endpoint: string,
    options: RequestInit = {}
): Promise<Response> => {
    const token = getAuthToken();

    const headers = {
        'Content-Type': 'application/json',
        ...(token && { 'Authorization': `Bearer ${token}` }),
        ...(options.headers || {})
    };

    return fetch(`${API_URL}${endpoint}`, {
        ...options,
        headers
    });
};

/**
 * User interface
 */
export interface User {
    id: number;
    first_name: string;
    last_name: string;
    email: string;
}

/**
 * Auth response interface
 */
export interface AuthResponse {
    success: boolean;
    message: string;
    token?: string;
    user?: User;
}
