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
    const [isSubmitting, setIsSubmitting] = useState(false);

    const {
        register,
        handleSubmit,
        watch,
        formState: { errors }
    } = useForm<SignUpFormData>();

    const password = watch('password');

    const onSubmit = async (data: SignUpFormData) => {
        setError('');
        setIsSubmitting(true);

        try {
            await signup(data);
            // Call onSuccess callback if provided
            if (onSuccess) {
                onSuccess();
            }
            // Redirect is handled in AuthContext
        } catch (err: any) {
            setError(err.message || 'Signup failed. Please try again.');
        } finally {
            setIsSubmitting(false);
        }
    };

    return (
        <form onSubmit={handleSubmit(onSubmit)} className={styles.authForm}>
            {error && <div className={styles.errorMessage}>{error}</div>}

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
                                /[0-9]/.test(value) || 'Must have number',
                            hasSpecialChar: (value) =>
                                /[!@#$%^&*(),.?":{}|<>]/.test(value) ||
                                'Must have special character'
                        }
                    })}
                    disabled={isSubmitting}
                />
                {errors.password && <span className={styles.fieldError}>{errors.password.message}</span>}
            </div>

            <div className={styles.formGroup}>
                <label htmlFor="confirm_password">Confirm Password</label>
                <input
                    id="confirm_password"
                    type="password"
                    {...register('confirm_password', {
                        required: 'Please confirm your password',
                        validate: (value) => value === password || 'Passwords do not match'
                    })}
                    disabled={isSubmitting}
                />
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
