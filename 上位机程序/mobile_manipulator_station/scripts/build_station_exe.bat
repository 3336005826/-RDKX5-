@echo off
setlocal

cd /d %~dp0

where pyinstaller >nul 2>nul
if errorlevel 1 (
    echo pyinstaller not found. Please run:
    echo pip install pyinstaller
    pause
    exit /b 1
)

if exist build rmdir /s /q build
if exist dist rmdir /s /q dist

pyinstaller ^
  --noconfirm ^
  --windowed ^
  --name station_gui ^
  --add-data "..\config;config" ^
  station_gui.py

if errorlevel 1 (
    echo Build failed.
    pause
    exit /b 1
)

if not exist dist\station_gui\config mkdir dist\station_gui\config
copy /Y ..\config\station_client.yaml dist\station_gui\config\station_client.yaml >nul

echo.
echo Build completed.
echo Output:
echo %cd%\dist\station_gui
echo.
pause
endlocal
