@echo off
pip install -r Intro_AI/requirements.txt

:: Xóa màn hình terminal (clear)
cls

:: Chạy ứng dụng Python
python Intro_AI/src/app.py

:: Giữ cửa sổ mở sau khi thực thi xong
pause
