@echo off
echo ========================================
echo   Starting Backend Server
echo ========================================
echo.

cd backend

echo Checking Python version...
python --version
echo.

echo Starting FastAPI server on port 8000...
echo Backend will be available at: http://127.0.0.1:8000
echo Health check: http://127.0.0.1:8000/health
echo API docs: http://127.0.0.1:8000/docs
echo.
echo Press Ctrl+C to stop the server
echo ========================================
echo.

python -m uvicorn app:app --reload --port 8000

pause
