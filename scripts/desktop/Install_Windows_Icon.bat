@echo off
rem Create Desktop shortcut "SolCam" -> SolCam_UI_preview.bat (custom icon).
set "BAT=%~dp0SolCam_UI_preview.bat"
set "ICO=%~dp0solcam.ico"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$d=[Environment]::GetFolderPath('Desktop'); $s=(New-Object -ComObject WScript.Shell).CreateShortcut($d+'\SolCam.lnk'); $s.TargetPath=$env:BAT; $s.WorkingDirectory=(Split-Path $env:BAT); $s.IconLocation=$env:ICO; $s.Description='SolCam'; $s.Save()"
echo.
echo Done: SolCam shortcut created on Desktop.
pause
