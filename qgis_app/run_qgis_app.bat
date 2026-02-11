@echo off
set "QGIS_PATH=C:\Program Files\QGIS 3.40.13"
set "QGIS_PREFIX_PATH=%QGIS_PATH%"
set "PATH=%QGIS_PATH%\bin;%PATH%"

if not exist "%QGIS_PATH%\bin\python-qgis-ltr.bat" (
  echo QGIS launcher not found at "%QGIS_PATH%\bin\python-qgis-ltr.bat"
  exit /b 1
)

call "%QGIS_PATH%\bin\python-qgis-ltr.bat" "%~dp0main.py"
