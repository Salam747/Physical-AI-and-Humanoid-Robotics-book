/**
 * Login Form Component
 * User login form with validation
 */

import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './styles.module.css';

interface LoginFormData {
    email: string;
    password: string;
}

interface LoginFormProps {
    onSuccess?: () => void;
}

const LoginForm: React.FC<LoginFormProps> = ({ onSuccess }) => {
    const { login } = useAuth();
    const [error, setError] = useState<string>('');
    const [isSubmitting, setIsSubmitting] = useState(false);

    const {
        register,
        handleSubmit,
        formState: { errors }
    } = useForm<LoginFormData>();

    const onSubmit = async (data: LoginFormData) => {
        setError('');
        setIsSubmitting(true);

        try {
            await login(data.email, data.password);
            // Call onSuccess callback if provided
            if (onSuccess) {
                onSuccess();
            }
            // Redirect is handled in AuthContext
        } catch (err: any) {
            setError(err.message || 'Login failed. Please check your credentials.');
        } finally {
            setIsSubmitting(false);
        }
    };

    return (
        <form onSubmit={handleSubmit(onSubmit)} className={styles.authForm}>
            {error && <div className={styles.errorMessage}>{error}</div>}

            <div className={styles.formGroup}>
                <label htmlFor="email">Email</label>
                <input
                    id="email"
                    type="email"
                    {...register('email', {
                        required: 'Email is required',
                        pattern: {
                            value: /^[A-Z0-9._%+-]+@[A-Z0-9.-]+\.[A-Z]{2,}$/i,
                            message: 'Invalid email address'
                        }
                    })}
                    disabled={isSubmitting}
                />
                {errors.email && <span className={styles.fieldError}>{errors.email.message}</span>}
            </div>

            <div className={styles.formGroup}>
                <label htmlFor="password">Password</label>
                <input
                    id="password"
                    type="password"
                    {...register('password', {
                        required: 'Password is required'
                    })}
                    disabled={isSubmitting}
                />
                {errors.password && <span className={styles.fieldError}>{errors.password.message}</span>}
            </div>

            <button type="submit" className={styles.submitButton} disabled={isSubmitting}>
                {isSubmitting ? 'Logging in...' : 'Log In'}
            </button>
        </form>
    );
};

export default LoginForm;
