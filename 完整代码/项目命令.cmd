@echo off
setlocal
chcp 65001 >nul
pushd "%~dp0"
if exist "%~dp0runtime\python.exe" (
  "%~dp0runtime\python.exe" -B -X utf8 "%~dp0run.py" %*
) else (
  python -B -X utf8 "%~dp0run.py" %*
)
set "UAV_EXIT=%ERRORLEVEL%"
popd
exit /b %UAV_EXIT%
