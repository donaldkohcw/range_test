@echo off
setlocal

:: Set IDF_PATH
set IDF_PATH=%USERPROFILE%\esp\v5.3.2\esp-idf
echo IDF_PATH set to: %IDF_PATH%

:: Define paths
set NVS_GEN_SCRIPT=%IDF_PATH%\components\nvs_flash\nvs_partition_generator\nvs_partition_gen.py
set CSV_FILE=nvs.csv
set OUTPUT_BIN=nvs.bin
set PARTITION_SIZE=0x4000

:: Check if Python is installed
where python >nul 2>&1
if %errorlevel% neq 0 (
    echo Python not found! Please install Python and try again.
    exit /b 1
)

:: Run the NVS Partition Generator script
python "%NVS_GEN_SCRIPT%" generate %CSV_FILE% %OUTPUT_BIN% %PARTITION_SIZE%

:: Check if the binary was created successfully
if exist %OUTPUT_BIN% (
    echo Successfully generated %OUTPUT_BIN%
) else (
    echo Failed to generate %OUTPUT_BIN%
    exit /b 1
)

endlocal
