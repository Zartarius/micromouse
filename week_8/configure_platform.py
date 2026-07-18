import platform

Import("env")

system = platform.system()

if system in {"Darwin", "Linux"}:
    env.Replace(BOARD="nanoatmega328")
    env.Replace(UPLOAD_SPEED="115200")

elif system in {"Windows"}:
    env.Replace(BOARD="nanoatmega328new")
    env.Replace(UPLOAD_PORT="COM10")
    env.Replace(UPLOAD_SPEED="115200")

else:
    print(f"configure_platform.py: unsupported platform '{system}', using defaults from platformio.ini")