@echo off

cd Intro_AI

pip install -r requirements.txt

:: Xóa màn hình terminal (clear)
cls

:: Chạy ứng dụng Python
python src/app.py

:: Giữ cửa sổ mở sau khi thực thi xong
pause
