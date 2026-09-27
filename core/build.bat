@echo off
rem Build setupcore and run its suites. Usage: build.bat [Debug]
setlocal
set CONFIG=%1
if "%CONFIG%"=="" set CONFIG=Release
cmake -S "%~dp0." -B "%~dp0build" || exit /b 1
cmake --build "%~dp0build" --config %CONFIG% || exit /b 1
"%~dp0build\%CONFIG%\setupcore_tests.exe" || exit /b 1
"%~dp0build\%CONFIG%\setupcore_search_tests.exe" || exit /b 1
echo.
echo setupcore: %~dp0build\%CONFIG%\setupcore.exe
