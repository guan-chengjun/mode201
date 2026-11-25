@echo off
rem 启动本地静态服务器（ES Module 需通过 http 访问，不能直接双击 html）
cd /d %~dp0
echo 打开浏览器访问: http://localhost:8000
python -m http.server 8000
pause
