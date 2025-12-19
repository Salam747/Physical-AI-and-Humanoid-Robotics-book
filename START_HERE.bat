@echo off
color 0A
echo.
echo ========================================
echo   Physical AI ^& Robotics Book
echo   Complete Application Launcher
echo ========================================
echo.
echo This will start BOTH backend and frontend servers
echo in separate windows.
echo.
echo Backend:  http://127.0.0.1:8000
echo Frontend: http://localhost:3002
echo.
echo Press any key to continue...
pause > nul

echo.
echo Starting Backend Server...
start "Backend Server" cmd /k "cd /d "%~dp0" && start-backend.bat"

echo Waiting 5 seconds for backend to initialize...
timeout /t 5 /nobreak > nul

echo.
echo Starting Frontend Server...
start "Frontend Server" cmd /k "cd /d "%~dp0" && start-frontend.bat"

echo.
echo ========================================
echo   Servers Starting!
echo ========================================
echo.
echo Backend:  http://127.0.0.1:8000/health
echo Frontend: http://localhost:3002
echo.
echo Login with:
echo   Email:    test@example.com
echo   Password: Test@123
echo.
echo Both servers are running in separate windows.
echo Close those windows to stop the servers.
echo.
echo Press any key to open the application in browser...
pause > nul

start http://localhost:3002

echo.
echo Application opened in browser!
echo.
pause
