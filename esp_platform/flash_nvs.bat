@echo off
setlocal

REM Set IDF_PATH
set IDF_PATH=%USERPROFILE%\esp\v5.3.2\esp-idf
echo IDF_PATH set to: %IDF_PATH%

REM Set the paths
set NVS_BIN_FILE=%CD%\nvs.bin
set COM_PORT=COM7  

echo Flashing NVS partition...
python %IDF_PATH%\components\esptool_py\esptool\esptool.py --chip esp32h2 --port %COM_PORT% write_flash 0x9000 %NVS_BIN_FILE%

echo NVS partition flashed successfully.

endlocal
pause
