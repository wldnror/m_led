from imu import MPU6050
import time
from machine import Pin, I2C
import neopixel
import math

# I2C 설정
i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)
mpu = MPU6050(i2c)

# NeoPixel LED 설정
pixel_pin = Pin(22)
num_pixels = 84
pixels = neopixel.NeoPixel(pixel_pin, num_pixels)

# 스위치 핀 설정
switch_pins = [Pin(10, Pin.IN, Pin.PULL_DOWN),
               Pin(11, Pin.IN, Pin.PULL_DOWN),
               Pin(12, Pin.IN, Pin.PULL_DOWN),
               Pin(13, Pin.IN, Pin.PULL_DOWN)]

# LED 색상 설정 (기본: 주황색)
colors = [(255, 120, 0),  # 주황색 (기본 색상)
          (255, 0, 0),    # 빨간색
          (0, 255, 0),    # 초록색
          (0, 0, 255),    # 파란색
          (75, 0, 130),   # 남색
          (238, 130, 238), # 보라색
          "rainbow"]      # 무지개 모드

color_index = 0  # 현재 색상 인덱스
rainbow_index = 0  # 무지개 효과 인덱스

def calculate_angle(acc_x, acc_y, acc_z):
    angle_x = math.atan2(acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 180 / math.pi
    angle_y = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)) * 180 / math.pi
    return angle_x, angle_y

def clear_leds():
    for i in range(num_pixels):
        pixels[i] = (0, 0, 0)
    pixels.write()

def display_leds(direction, on, rainbow_index):
    clear_leds()  # 모든 LED를 초기화
    brightness = 50 if on else 0

    def apply_rainbow_effect(start, end):
        for i in range(start, end):
            pixel_index = (i + rainbow_index) & 255
            rainbow_color = wheel(pixel_index)
            adjusted_color = (rainbow_color[0] * brightness // 255, rainbow_color[1] * brightness // 255, rainbow_color[2] * brightness // 255)
            pixels[i] = adjusted_color if on else (0, 0, 0)

    if colors[color_index] == "rainbow":
        rainbow = True
        color = (0, 0, 0)
    else:
        rainbow = False
        color = colors[color_index]

    adjusted_color = (color[0] * brightness // 255, color[1] * brightness // 255, color[2] * brightness // 255)

    if direction == "left" or direction == "both":
        group2_start, group2_end = 21, 42  # 2번 그룹
        if rainbow:
            apply_rainbow_effect(group2_start, group2_end)
        else:
            for i in range(group2_start, group2_end):
                pixels[i] = adjusted_color if on else (0, 0, 0)

    if direction == "right" or direction == "both":
        group3_start, group3_end = 42, 63  # 3번 그룹
        if rainbow:
            apply_rainbow_effect(group3_start, group3_end)
        else:
            for i in range(group3_start, group3_end):
                pixels[i] = adjusted_color if on else (0, 0, 0)

    pixels.write()  # 2번과 3번 그룹 동시 쓰기
    time.sleep(0.1)  # 0.1초 지연

    if direction == "left" or direction == "both":
        group1_start, group1_end = 0, 21  # 1번 그룹
        if rainbow:
            apply_rainbow_effect(group1_start, group1_end)
        else:
            for i in range(group1_start, group1_end):
                pixels[i] = adjusted_color if on else (0, 0, 0)

    if direction == "right" or direction == "both":
        group4_start, group4_end = 63, 84  # 4번 그룹
        if rainbow:
            apply_rainbow_effect(group4_start, group4_end)
        else:
            for i in range(group4_start, group4_end):
                pixels[i] = adjusted_color if on else (0, 0, 0)

    pixels.write()  # 1번과 4번 그룹 동시 쓰기

def wheel(pos):
    # 무지개 색상을 생성하는 함수
    if pos < 85:
        return (pos * 3, 255 - pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (255 - pos * 3, 0, pos * 3)
    else:
        pos -= 170
        return (0, pos * 3, 255 - pos * 3)

def control_leds():
    flash_interval = 350  # LED 깜빡임 간격을 350ms로 설정
    last_flash_time = time.ticks_ms()
    active_tilt = None
    led_on = False
    cycle_complete = True
    low_speed_threshold = 2  # 임계 각속도 값 설정 (예: 저속)
    high_threshold = 20  # 높은 임계값
    low_threshold = 10  # 낮은 임계값, hysteresis 적용
    auto_mode = True  # 초기 모드는 자동 모드
    manual_direction = None
    hazard_lights_on = False  # 비상등 상태
    repeat_count = 5  # 수동 모드 반복 횟수 초기화

    global color_index  # 전역 변수 접근
    global rainbow_index  # 전역 변수 접근

    # serve.py 파일 읽기
    with open('serve.py') as f:
        serve_script = f.read()

    # 별도의 스레드 또는 비동기 방식으로 serve.py 코드 실행
    import _thread
    def run_serve_script():
        exec(serve_script)

    _thread.start_new_thread(run_serve_script, ())

    while True:
        if switch_pins[2].value() == 1:  # 3번 스위치가 눌렸을 때 모드 전환
            auto_mode = not auto_mode
            manual_direction = None  # 수동 동작 중단
            hazard_lights_on = False  # 비상등 중단
            clear_leds()  # LED 초기화
            time.sleep(0.5)  # 버튼 debounce를 위한 지연

        if auto_mode:
            if switch_pins[1].value() == 1:  # 2번 스위치가 눌렸을 때 색상 변경 (이전 색상)
                color_index = (color_index - 1) % len(colors)
                time.sleep(0.5)  # 버튼 debounce를 위한 지연
            elif switch_pins[3].value() == 1:  # 4번 스위치가 눌렸을 때 색상 변경 (다음 색상)
                color_index = (color_index + 1) % len(colors)
                time.sleep(0.5)  # 버튼 debounce를 위한 지연

            if switch_pins[0].value() == 1:  # 1번 스위치가 눌렸을 때 기본 색상으로 변경
                color_index = 0  # 기본 색상 인덱스 (주황색)
                time.sleep(0.5)  # 버튼 debounce를 위한 지연

        if not auto_mode and switch_pins[0].value() == 1:  # 수동 모드에서 1번 스위치가 눌렸을 때 비상등 토글
            hazard_lights_on = not hazard_lights_on
            clear_leds()
            time.sleep(0.5)  # 버튼 debounce를 위한 지연

        if hazard_lights_on:
            if time.ticks_diff(time.ticks_ms(), last_flash_time) > flash_interval:
                last_flash_time = time.ticks_ms()
                led_on = not led_on
                display_leds("both", led_on, rainbow_index)
            continue  # 비상등이 켜져 있을 때는 다른 기능 무시

        if auto_mode:
            acc_x, acc_y, acc_z = mpu.accel.x, mpu.accel.y, mpu.accel.z
            gyro_y = mpu.gyro.y
            _, angle_y = calculate_angle(acc_x, acc_y, acc_z)

            print("Current tilt angle Y-axis:", angle_y, "Gyro Y-axis speed:", gyro_y)

            if cycle_complete:
                if angle_y > high_threshold:
                    direction = "right"
                elif angle_y < -high_threshold:
                    direction = "left"
                elif abs(gyro_y) < low_speed_threshold:
                    direction = "both"  # 저속일 때 양쪽 LED 깜빡임
                else:
                    direction = None

            if direction:
                if active_tilt != direction:
                    active_tilt = direction
                    led_on = True
                    last_flash_time = time.ticks_ms()  # LED 켜기 시작 시간 리셋
                    display_leds(direction, led_on, rainbow_index)
                    cycle_complete = False
                elif time.ticks_diff(time.ticks_ms(), last_flash_time) > flash_interval:
                    last_flash_time = time.ticks_ms()
                    led_on = not led_on
                    display_leds(direction, led_on, rainbow_index)
                    if not led_on:
                        cycle_complete = True

            else:
                if active_tilt:
                    clear_leds()
                    active_tilt = None
                    led_on = False
                    cycle_complete = True

        else:  # 수동 모드
            if manual_direction is None:
                if switch_pins[3].value() == 1:  # 4번 스위치가 눌렸을 때 좌회전
                    manual_direction = "left"
                    repeat_count = 5  # 반복 횟수 초기화
                elif switch_pins[1].value() == 1:  # 2번 스위치가 눌렸을 때 우회전
                    manual_direction = "right"
                    repeat_count = 5  # 반복 횟수 초기화

            if manual_direction:
                for i in range(repeat_count):  # 수동 모드 반복
                    if switch_pins[2].value() == 1:  # 동작 중 3번 스위치 눌림 감지
                        auto_mode = True
                        manual_direction = None
                        clear_leds()
                        break
                    if switch_pins[3].value() == 1 and manual_direction != "left":  # 동작 중 4번 스위치 눌림 감지
                        manual_direction = "left"
                        repeat_count = 5  # 반복 횟수 초기화
                        clear_leds()
                        break
                    if switch_pins[1].value() == 1 and manual_direction != "right":  # 동작 중 2번 스위치 눌림 감지
                        manual_direction = "right"
                        repeat_count = 5  # 반복 횟수 초기화
                        clear_leds()
                        break
                    display_leds(manual_direction, True, rainbow_index)
                    for _ in range(int(flash_interval / 10)):
                        if switch_pins[2].value() == 1:  # 동작 중 3번 스위치 눌림 감지
                            auto_mode = True
                            manual_direction = None
                            clear_leds()
                            break
                        if switch_pins[3].value() == 1 and manual_direction != "left":  # 동작 중 4번 스위치 눌림 감지
                            manual_direction = "left"
                            repeat_count = 5  # 반복 횟수 초기화
                            clear_leds()
                            break
                        if switch_pins[1].value() == 1 and manual_direction != "right":  # 동작 중 2번 스위치 눌림 감지
                            manual_direction = "right"
                            repeat_count = 5  # 반복 횟수 초기화
                            clear_leds()
                            break
                        time.sleep(0.01)
                    display_leds(manual_direction, False, rainbow_index)
                    for _ in range(int(flash_interval / 10)):
                        if switch_pins[2].value() == 1:  # 동작 중 3번 스위치 눌림 감지
                            auto_mode = True
                            manual_direction = None
                            clear_leds()
                            break
                        if switch_pins[3].value() == 1 and manual_direction != "left":  # 동작 중 4번 스위치 눌림 감지
                            manual_direction = "left"
                            repeat_count = 5  # 반복 횟수 초기화
                            clear_leds()
                            break
                        if switch_pins[1].value() == 1 and manual_direction != "right":  # 동작 중 2번 스위치 눌림 감지
                            manual_direction = "right"
                            repeat_count = 5  # 반복 횟수 초기화
                            clear_leds()
                            break
                        time.sleep(0.01)
                clear_leds()
                manual_direction = None

        # 무지개 효과 업데이트
        if colors[color_index] == "rainbow":
            rainbow_index = (rainbow_index + 1) % 256

        time.sleep(0.05)

# 메인 함수 호출
control_leds()

