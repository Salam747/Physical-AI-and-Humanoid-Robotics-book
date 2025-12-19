/**
 * Sign Up Form Component
 * User registration form with validation
 */

import React, { useState } from 'react';
import { useForm } from 'react-hook-form';
import { useAuth } from '@site/src/context/AuthContext';
import styles from './styles.module.css';

interface SignUpFormData {
    first_name: string;
    last_name: string;
    email: string;
    password: string;
    confirm_password: string;
}

interface SignUpFormProps {
    onSuccess?: () => void;
}

const SignUpForm: React.FC<SignUpFormProps> = ({ onSuccess }) => {
    const { signup } = useAuth();
    const [error, setError] = useState<string>('');
    const [success, setSuccess] = useState<string>('');
    const [isSubmitting, setIsSubmitting] = useState(false);
    const [showPassword, setShowPassword] = useState(false);
    const [showConfirmPassword, setShowConfirmPassword] = useState(false);

    const {
        register,
        handleSubmit,
        watch,
        formState: { errors }
    } = useForm<SignUpFormData>();

    const password = watch('password');

    const onSubmit = async (data: SignUpFormData) => {
        setError('');
        setSuccess('');
        setIsSubmitting(true);

        try {
            await signup(data);
            setSuccess('✓ Account created successfully! Opening chatbot...');

            // Wait a moment to show success message
            setTimeout(() => {
                if (onSuccess) {
                    onSuccess();
                }
            }, 800);
            // Redirect is handled in AuthContext
        } catch (err: any) {
            console.error('Signup error:', err);
            const errorMessage = err?.message || err?.toString() || 'Signup failed. Please try again.';
            setError(errorMessage);
        } finally {
            setIsSubmitting(false);
        }
    };

    return (
        <form onSubmit={handleSubmit(onSubmit)} className={styles.authForm}>
            {error && <div className={styles.errorMessage}>{error}</div>}
            {success && <div className={styles.successMessage}>{success}</div>}

            <div className={styles.formRow}>
                <div className={styles.formGroup}>
                    <label htmlFor="first_name">First Name</label>
                    <input
                        id="first_name"
                        type="text"
                        {...register('first_name', {
                            required: 'First name is required',
                            minLength: {
                                value: 2,
                                message: 'First name must be at least 2 characters'
                            },
                            maxLength: {
                                value: 50,
                                message: 'First name must be at most 50 characters'
                            },
                            pattern: {
                                value: /^[a-zA-Z\s\-']+$/,
                                message: 'First name can only contain letters, spaces, hyphens, and apostrophes'
                            }
                        })}
                        disabled={isSubmitting}
                    />
                    {errors.first_name && (
                        <span className={styles.fieldError}>{errors.first_name.message}</span>
                    )}
                </div>

                <div className={styles.formGroup}>
                    <label htmlFor="last_name">Last Name</label>
                    <input
                        id="last_name"
                        type="text"
                        {...register('last_name', {
                            required: 'Last name is required',
                            minLength: {
                                value: 2,
                                message: 'Last name must be at least 2 characters'
                            },
                            maxLength: {
                                value: 50,
                                message: 'Last name must be at most 50 characters'
                            },
                            pattern: {
                                value: /^[a-zA-Z\s\-']+$/,
                                message: 'Last name can only contain letters, spaces, hyphens, and apostrophes'
                            }
                        })}
                        disabled={isSubmitting}
                    />
                    {errors.last_name && (
                        <span className={styles.fieldError}>{errors.last_name.message}</span>
                    )}
                </div>
            </div>

            <div className={styles.formGroup}>
                <label htmlFor="email">Email</label>
                <input
                    id="email"
                    type="email"
                    placeholder="your@email.com"
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
                <div className={styles.passwordWrapper}>
                    <input
                        id="password"
                        type={showPassword ? 'text' : 'password'}
                        placeholder="Create a strong password"
                        {...register('password', {
                            required: 'Password is required',
                            minLength: {
                                value: 6,
                                message: 'Password must be 6-14 characters'
                            },
                            maxLength: {
                                value: 14,
                                message: 'Password must be 6-14 characters'
                            },
                            validate: {
                                hasUpperCase: (value) =>
                                    /[A-Z]/.test(value) || 'Must have uppercase letter',
                                hasLowerCase: (value) =>
                                    /[a-z]/.test(value) || 'Must have lowercase letter',
                                hasNumber: (value) =>
                                    /[0-9]/.test(value) || 'Must have number'
                            }
                        })}
                        disabled={isSubmitting}
                        style={{ paddingRight: '45px' }}
                    />
                    <button
                        type="button"
                        className={styles.passwordToggle}
                        onClick={() => setShowPassword(!showPassword)}
                        tabIndex={-1}
                    >
                        {showPassword ? (
                            <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13.875 18.825A10.05 10.05 0 0112 19c-4.478 0-8.268-2.943-9.543-7a9.97 9.97 0 011.563-3.029m5.858.908a3 3 0 114.243 4.243M9.878 9.878l4.242 4.242M9.88 9.88l-3.29-3.29m7.532 7.532l3.29 3.29M3 3l3.59 3.59m0 0A9.953 9.953 0 0112 5c4.478 0 8.268 2.943 9.543 7a10.025 10.025 0 01-4.132 5.411m0 0L21 21" />
                            </svg>
                        ) : (
                            <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
                            </svg>
                        )}
                    </button>
                </div>
                {errors.password && <span className={styles.fieldError}>{errors.password.message}</span>}
            </div>

            <div className={styles.formGroup}>
                <label htmlFor="confirm_password">Confirm Password</label>
                <div className={styles.passwordWrapper}>
                    <input
                        id="confirm_password"
                        type={showConfirmPassword ? 'text' : 'password'}
                        placeholder="Confirm your password"
                        {...register('confirm_password', {
                            required: 'Please confirm your password',
                            validate: (value) => value === password || 'Passwords do not match'
                        })}
                        disabled={isSubmitting}
                        style={{ paddingRight: '45px' }}
                    />
                    <button
                        type="button"
                        className={styles.passwordToggle}
                        onClick={() => setShowConfirmPassword(!showConfirmPassword)}
                        tabIndex={-1}
                    >
                        {showConfirmPassword ? (
                            <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13.875 18.825A10.05 10.05 0 0112 19c-4.478 0-8.268-2.943-9.543-7a9.97 9.97 0 011.563-3.029m5.858.908a3 3 0 114.243 4.243M9.878 9.878l4.242 4.242M9.88 9.88l-3.29-3.29m7.532 7.532l3.29 3.29M3 3l3.59 3.59m0 0A9.953 9.953 0 0112 5c4.478 0 8.268 2.943 9.543 7a10.025 10.025 0 01-4.132 5.411m0 0L21 21" />
                            </svg>
                        ) : (
                            <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" stroke="currentColor">
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
                                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
                            </svg>
                        )}
                    </button>
                </div>
                {errors.confirm_password && (
                    <span className={styles.fieldError}>{errors.confirm_password.message}</span>
                )}
            </div>

            <button type="submit" className={styles.submitButton} disabled={isSubmitting}>
                {isSubmitting ? 'Creating Account...' : 'Sign Up'}
            </button>
        </form>
    );
};

export default SignUpForm;
