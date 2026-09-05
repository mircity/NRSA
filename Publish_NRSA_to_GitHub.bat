@echo off
setlocal
title Publish NRSA to GitHub

echo.
echo ============================================
echo       NRSA - GitHub Publish Tool
echo ============================================
echo.
echo This tool will upload the project folder
echo containing this BAT file to:
echo https://github.com/mircity/NRSA.git
echo.
pause

REM Move to the folder containing this BAT file
cd /d "%~dp0"

REM Check for Git
where git >nul 2>nul
if errorlevel 1 (
    echo.
    echo Git is not installed.
    echo Please install Git for Windows from:
    echo https://git-scm.com/download/win
    echo Then run this file again.
    pause
    exit /b 1
)

REM Initialize repository if necessary
if not exist ".git" (
    git init
)

git add .

git diff --cached --quiet
if errorlevel 1 (
    git commit -m "Initial commit - NRSA project"
) else (
    echo No new changes to commit.
)

git branch -M main

git remote get-url origin >nul 2>nul
if errorlevel 1 (
    git remote add origin https://github.com/mircity/NRSA.git
) else (
    git remote set-url origin https://github.com/mircity/NRSA.git
)

echo.
echo Uploading project to GitHub...
echo A browser login window may open.
echo Please sign in to GitHub if requested.
echo.

git push -u origin main

if errorlevel 1 (
    echo.
    echo ============================================
    echo Upload failed.
    echo If the repository contains an existing README,
    echo contact me and I will help fix it.
    echo ============================================
) else (
    echo.
    echo ============================================
    echo SUCCESS! Your NRSA project is on GitHub.
    echo https://github.com/mircity/NRSA
    echo ============================================
)

echo.
pause
