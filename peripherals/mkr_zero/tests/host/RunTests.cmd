@echo off
setlocal enabledelayedexpansion

rem Host tests for lib\SwitchFunctions.cpp, built with MSVC.
rem
rem   RunTests.cmd        build and run
rem
rem Exit code is the number of failing checks, so this is usable as a gate.
rem
rem There is a Makefile beside this file for g++ / clang++ - use that on the
rem Orin Nano or any Linux box. This script exists because the Windows machine
rem this firmware is normally built from has no g++ at all: arduino-cli ships
rem only arm-none-eabi, which cross-compiles and cannot run what it produces.
rem Tests that cannot be executed where the code is written do not get run.
rem
rem THIS DIRECTORY IS FIRST ON THE INCLUDE PATH ON PURPOSE. lib\SwitchFunctions.h
rem includes <Arduino.h> with angle brackets, so .\Arduino.h shadows the real
rem one and the module under test compiles UNMODIFIED - no #ifdef, no test-only
rem build of the firmware, and no second copy of the logic to drift out of sync.
rem
rem NOTE: keep this file plain ASCII - cmd.exe parses it in the OEM codepage.

set "VCVARS="
for %%E in (Community Professional Enterprise BuildTools) do (
    for %%V in (18 2022 2019) do (
        if not defined VCVARS (
            if exist "%ProgramFiles%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                set "VCVARS=%ProgramFiles%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat"
            )
            if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                set "VCVARS=%ProgramFiles(x86)%\Microsoft Visual Studio\%%V\%%E\VC\Auxiliary\Build\vcvars64.bat"
            )
        )
    )
)

if not defined VCVARS (
    echo No MSVC installation found.
    echo Install "Desktop development with C++", or use the Makefile with g++.
    exit /b 1
)

rem vcvars prints a harmless complaint when vswhere is absent; it still works.
call "%VCVARS%" >nul 2>&1

rem NOT named LIB. That is MSVC's own library search path, exported by vcvars
rem above, and overwriting it makes the compile fail at link with
rem "cannot open file 'LIBCMT.lib'" - which reads as a broken toolchain
rem installation rather than as this script standing on it.
for %%I in ("%~dp0.") do set "TESTS=%%~fI"
for %%I in ("%TESTS%\..\..\lib") do set "LIBDIR=%%~fI"

cd /d "%TESTS%"

cl /nologo /std:c++14 /W4 /EHsc /I"%TESTS%" /I"%LIBDIR%" ^
   arduino_stub.cpp switch_tests.cpp "%LIBDIR%\SwitchFunctions.cpp" ^
   /Fe:switch_tests.exe
if errorlevel 1 (
    del /q *.obj 2>nul
    echo Build failed.
    exit /b 1
)

echo.
.\switch_tests.exe
set "RC=%ERRORLEVEL%"

del /q *.obj 2>nul
exit /b %RC%
