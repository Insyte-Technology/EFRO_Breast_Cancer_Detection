@echo off
rem Starts the AD9106 GUI. First run creates a local Python environment (.venv)
rem and installs the requirements; later runs start immediately.
setlocal
cd /d "%~dp0"

if not exist ".venv\installed.ok" (
    echo Setting up Python environment, this takes a minute the first time...
    if not exist ".venv\Scripts\python.exe" (
        py -3 -m venv .venv 2>nul || python -m venv .venv
    )
    if not exist ".venv\Scripts\python.exe" (
        echo.
        echo Python 3 was not found. Install it from https://www.python.org and run this again.
        pause
        exit /b 1
    )
    ".venv\Scripts\python.exe" -m pip install --upgrade pip
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt || (
        echo.
        echo Installing the requirements failed.
        pause
        exit /b 1
    )
    echo ok> ".venv\installed.ok"
)

start "" ".venv\Scripts\pythonw.exe" ad9106_gui.py %*
