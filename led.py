from machine import I2C, Pin
import neopixel
import time
from imu import MPU6050

# I2C 설정 (I2C0 사용, 핀 8과 9)
i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)

# I2C 스캔
devices = i2c.scan()
print("I2C devices found:", [hex(device) for device in devices])

if 0x68 not in devices:
    raise Exception("MPU6050 not detected on I2C bus")

mpu = MPU6050(i2c)

# NeoPixel LED 설정
pixel_pin = Pin(22)
num_pixels = 144
pixels = neopixel.NeoPixel(pixel_pin, num_pixels)

# 무지개 색 정의
rainbow_colors = [(255, 0, 0), (255, 127, 0), (255, 255, 0), 
                  (0, 255, 0), (0, 0, 255), (75, 0, 130), (148, 0, 211)]

def clear_leds():
    for i in range(num_pixels):
        pixels[i] = (0, 0, 0)
    pixels.write()

def rainbow_effect(step):
    for i in range(num_pixels):
        idx = (i + step) % len(rainbow_colors)
        pixels[i] = rainbow_colors[idx]
    pixels.write()

def blink_leds(times=5):
    for _ in range(times):
        clear_leds()
        for i in range(0, num_pixels, 2):
            pixels[i] = (255, 150, 0)  # 새로운 색상
        pixels.write()
        time.sleep(0.4)
        clear_leds()
        time.sleep(0.4)

def main():
    previous_acc_x = mpu.accel.x  # 초기 가속도 값 저장
    threshold = 0.5  # 변화 임계값 설정
    debounce_time = 2  # 연속 감지 방지 시간 (초)
    last_tap_time = time.ticks_ms() - debounce_time * 1000
    step = 0

    while True:
        start_time = time.ticks_ms()

        # 무지개 효과 업데이트
        rainbow_effect(step)
        step = (step + 1) % num_pixels

        # 충격 감지
        current_acc_x = mpu.accel.x
        change_in_x = abs(current_acc_x - previous_acc_x)
        print(f"Current X: {current_acc_x}, Change in X: {change_in_x}")
        
        current_time = time.ticks_ms()
        if change_in_x > threshold and time.ticks_diff(current_time, last_tap_time) > debounce_time * 1000:
            print("Significant change detected, blinking LEDs")
            blink_leds()
            last_tap_time = time.ticks_ms()  # 깜빡임 후 시간 업데이트
        
        previous_acc_x = current_acc_x

        # 무지개 효과와 충격 감지 사이에 짧은 지연을 추가
        elapsed_time = time.ticks_diff(time.ticks_ms(), start_time)
        time.sleep(max(0, 0.05 - (elapsed_time / 1000)))  # 0.05초 간격 유지

if __name__ == "__main__":
    main()

