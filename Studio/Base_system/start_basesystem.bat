@echo off
REM ============================================================
REM  Base System V1.2 - one-command launcher
REM  1) loads the frontend Docker image (only if missing)
REM  2) starts the web UI via docker-compose
REM  3) opens the browser
REM  4) runs the Python/STM bridge (main_v1_2.exe) in foreground
REM ============================================================
setlocal
cd /d "%~dp0"

echo [1/4] Checking Docker is running...
docker info >nul 2>&1
if errorlevel 1 (
    echo  ERROR: Docker Desktop is not running. Start it and try again.
    pause
    exit /b 1
)

echo [2/4] Loading frontend image (skips if already present)...
docker image inspect my-frontend:latest >nul 2>&1
if errorlevel 1 (
    docker load -i "frontend-image_v1_2.tar"
) else (
    echo  Image my-frontend:latest already loaded - skipping.
)

echo [3/4] Starting web UI container...
docker-compose up -d
if errorlevel 1 (
    echo  ERROR: docker-compose failed. Check that port 3000 is free.
    pause
    exit /b 1
)

REM give the container a moment, then open the dashboard
timeout /t 2 >nul
start "" "http://localhost:3000"

echo [4/4] Starting STM bridge (main_v1_2.exe)...
echo  Leave this window open. Close it (Ctrl+C) to stop the bridge.
echo  To stop the web UI later, run:  docker-compose down
echo ------------------------------------------------------------
main_v1_2.exe

endlocal
