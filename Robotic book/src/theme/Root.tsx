/**
 * Root Component
 * Wraps the entire app with AuthProvider
 */

import React, { ReactNode } from 'react';
import { AuthProvider } from '@site/src/context/AuthContext';

interface RootProps {
    children: ReactNode;
}

export default function Root({ children }: RootProps): JSX.Element {
    return <AuthProvider>{children}</AuthProvider>;
}
