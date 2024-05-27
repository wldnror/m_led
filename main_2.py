import machine
import time
import uos

# D+ 핀 설정 (보드에 맞게 수정 필요)
usb_connected_pin = machine.Pin(2, machine.Pin.IN, machine.Pin.PULL_UP)

# USB 연결 상태 확인 함수 (1V 이하인지 확인)
def is_usb_connected():
    return usb_connected_pin.value() == 0  # 풀업 저항 사용 시, 낮은 신호가 연결됨을 의미

# 파일 시스템 내용 지우기 함수
def delete_files(path):
    for entry in uos.ilistdir(path):
        entry_name = entry[0]
        entry_type = entry[1]
        full_path = f"{path}/{entry_name}"
        
        if entry_type == 0x4000:  # Directory
            try:
                delete_files(full_path)  # Recursively delete directory contents
                uos.rmdir(full_path)
            except Exception as e:
                print(f"Error deleting directory {entry_name}: {e}")
        elif entry_type == 0x8000:  # File
            try:
                uos.remove(full_path)
            except Exception as e:
                print(f"Error deleting file {entry_name}: {e}")

# 주기적으로 상태 확인 및 동작 수행
while True:
    if is_usb_connected():
        print("USB connected. Deleting files...")
        delete_files("/")
        machine.reset()  # 시스템 리셋하여 초기화
    else:
        print("USB not connected.")
    
    time.sleep(1)  # 1초마다 상태 확인

