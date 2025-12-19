@echo off
echo ========================================
echo   Starting Frontend Server
echo ========================================
echo.

cd "Robotic book"

echo Checking Node.js version...
node --version
echo.

echo Checking npm version...
npm --version
echo.

echo Starting Docusaurus development server on port 3002...
echo Frontend will be available at: http://localhost:3002
echo.
echo Press Ctrl+C to stop the server
echo ========================================
echo.

npm start -- --port 3002

pause
