@echo off
setlocal

rem Validate SHA-256 hashes and RFC 3161 timestamps for Patent Search evidence.
cd /d "%~dp0"

where python >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is required to run evidence validation.
    echo Install Python 3 or run:
    echo   python evidence\validate_evidence.py
    exit /b 1
)

python "%~dp0evidence\validate_evidence.py" %*
exit /b %ERRORLEVEL%
