@echo off
setlocal
chcp 65001 >nul
pushd "%~dp0"
if not exist "runtime\python.exe" (
  python -B -X utf8 "%~dp0launch.py"
) else (
  "%~dp0runtime\python.exe" -B -X utf8 "%~dp0launch.py"
)
set "UAV_EXIT=%ERRORLEVEL%"
popd
if not "%UAV_EXIT%"=="0" pause
exit /b %UAV_EXIT%
