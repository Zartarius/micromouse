import platform
import datetime

Import("env")

system = platform.system()

if system == "Windows":
    env.Replace(UPLOAD_PORT="COM10")

current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
env.Append(CPPDEFINES=[("BUILD_TIMESTAMP", f'\\"{current_time}\\"')])
print(f">>> Successfully injected build timestamp: {current_time}")