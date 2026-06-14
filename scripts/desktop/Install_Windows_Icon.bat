@echo off
rem Create Desktop shortcut "SolCam UI" -> SolCam_UI_preview.bat (custom icon).
set "BAT=%~dp0SolCam_UI_preview.bat"
set "ICO=%~dp0solcam.ico"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$d=[Environment]::GetFolderPath('Desktop'); $s=(New-Object -ComObject WScript.Shell).CreateShortcut($d+'\SolCam UI.lnk'); $s.TargetPath=$env:BAT; $s.WorkingDirectory=(Split-Path $env:BAT); $s.IconLocation=$env:ICO; $s.Description='SolCam UI preview'; $s.Save()"
echo.
echo Done: SolCam UI shortcut created on Desktop.
pause
