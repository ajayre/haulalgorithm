@echo off
setlocal EnableDelayedExpansion

rem Validate SHA-256 hashes and RFC 3161 timestamps for Patent Search evidence.
cd /d "%~dp0"

set "OPENSSL=C:\Program Files\Git\usr\bin\openssl.exe"
if not exist "%OPENSSL%" (
    echo ERROR: OpenSSL not found at:
    echo   %OPENSSL%
    echo Install Git for Windows or update OPENSSL in this script.
    exit /b 1
)

if not exist "evidence\freetsa-cacert.pem" (
    echo ERROR: evidence\freetsa-cacert.pem not found.
    exit /b 1
)
if not exist "evidence\freetsa-tsa.crt" (
    echo ERROR: evidence\freetsa-tsa.crt not found.
    exit /b 1
)

set "PASS=0"
set "FAIL=0"

echo.
echo Patent Search Evidence Validation
echo =================================
echo.

for %%T in (evidence\*.tsr) do (
    set "BASE=%%~nT"
    set "TSR=%%T"
    set "TSQ=evidence\!BASE!.tsq"
    set "HASHFILE=evidence\!BASE!.sha256"

    if not exist "!TSQ!" (
        echo [FAIL] !BASE! - missing timestamp request: !TSQ!
        set /a FAIL+=1
    ) else if not exist "!HASHFILE!" (
        echo [FAIL] !BASE! - missing hash file: !HASHFILE!
        set /a FAIL+=1
    ) else (
        set /p "HASHLINE=" < "!HASHFILE!"
        set "EXPECTED=!HASHLINE:~0,64!"
        set "SOURCE=!HASHLINE:~66!"

        if not exist "!SOURCE!" (
            echo [FAIL] !SOURCE! - source file not found
            set /a FAIL+=1
        ) else (
            set "ACTUAL="
            for /f "usebackq skip=1 tokens=*" %%H in (`certutil -hashfile "!SOURCE!" SHA256 ^| findstr /v /i "CertUtil"`) do (
                set "ACTUAL=%%H"
            )
            set "ACTUAL=!ACTUAL: =!"

            if /i not "!ACTUAL!"=="!EXPECTED!" (
                echo [FAIL] !SOURCE! - SHA-256 mismatch
                echo         expected: !EXPECTED!
                echo         actual:   !ACTUAL!
                set /a FAIL+=1
            ) else (
                "%OPENSSL%" ts -verify -in "!TSR!" -queryfile "!TSQ!" -CAfile "evidence\freetsa-cacert.pem" -untrusted "evidence\freetsa-tsa.crt" >nul 2>&1
                if errorlevel 1 (
                    echo [FAIL] !SOURCE! - RFC 3161 timestamp verification failed
                    set /a FAIL+=1
                ) else (
                    echo [OK]   !SOURCE!
                    set /a PASS+=1
                )
            )
        )
    )
)

echo.
echo ---------------------------------
echo Passed: !PASS!   Failed: !FAIL!
echo ---------------------------------
echo.

if !FAIL! gtr 0 (
    exit /b 1
)
exit /b 0
