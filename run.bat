@echo off

cd intro_ai

pip install -r requirements.txt

:: Chạy ứng dụng Python
python src/app.py

:: Giữ cửa sổ mở sau khi thực thi xong
pause
