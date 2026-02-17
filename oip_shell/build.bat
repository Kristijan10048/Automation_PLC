@echo off
setlocal

:: Find CMake from Visual Studio installation
for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath 2^>nul`) do set "VS_PATH=%%i"

if defined VS_PATH (
    set "CMAKE=%VS_PATH%\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
) else (
    where cmake >nul 2>&1
    if %errorlevel%==0 (
        set "CMAKE=cmake"
    ) else (
        echo Error: Could not find CMake. Install CMake or Visual Studio with C++ tools.
        exit /b 1
    )
)

echo Using CMake: %CMAKE%
echo.

:: Configure
echo === Configuring ===
"%CMAKE%" -B build -G "Visual Studio 17 2022"
if %errorlevel% neq 0 (
    echo Configuration failed.
    exit /b 1
)

echo.

:: Build
echo === Building ===
"%CMAKE%" --build build --config Release
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b 1
)

echo.
echo === Build successful ===
echo Executable: build\Release\oip_shell.exe
