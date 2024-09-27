#!/usr/bin/env python3

import serial
import time
import rospy
from std_msgs.msg import Float64

class Rion:
    # Инициализация последовательного порта
    ser = serial.Serial(
        port='/dev/ttyUSB0',  # Укажите правильный порт USB
        baudrate=9600,
        timeout=1
    )

    def __init__(self):
        self.heading_publisher_ = rospy.Publisher('/heading', Float64, queue_size=1)
        print("Начинаем работу с компасом...")
        time.sleep(1)  # Небольшая задержка для стабилизации


    def send_command(self, cmd):
        self.ser.write(cmd)
        self.ser.flush()
        # print(f'Отправлена команда: {" ".join([f"{byte:02X}" for byte in cmd])}')

    def read_response(self, length, timeout=2):
        start_time = time.time()
        response = bytearray()

        while time.time() - start_time < timeout and len(response) < length:
            if self.ser.in_waiting > 0:
                response.extend(self.ser.read(self.ser.in_waiting))
        
        return response

    def request_data(self):
        # Команда для запроса данных углов (Pitch, Roll, Heading): 68 04 00 04 08
        request_cmd = bytearray([0x68, 0x04, 0x00, 0x04, 0x08])
        self.send_command(request_cmd)

        response = self.read_response(13)
        
        # Выводим сырые данные вне зависимости от их корректности
        if response:
            pass# print(f'Получен ответ (сырые данные): {" ".join([f"{byte:02X}" for byte in response])}')
        else:
            rospy.loginfo(print("Ошибка: ответ пустой или данные не получены."))

        if len(response) == 14:
            if response[3] == 0x84:
                self.parse_data(response)
            else:
                rospy.loginfo(print("Ошибка: неверный идентификатор ответа."))
        else:
            rospy.loginfo(print("Ошибка: данные от компаса не получены или ответ некорректный."))

    def parse_data(self, response):
        pitch = self.parse_angle(response[4], response[5], response[6])
        roll = self.parse_angle(response[7], response[8], response[9])
        heading = self.parse_angle(response[10], response[11], response[12], is_heading=True)
        self.heading_publisher_.publish(float(heading))
        # Вывод данных в консоль
        # print(f"Pitch: {pitch:+.1f}°, Roll: {roll:+.1f}°, Heading: {heading:.1f}°\n")

    def parse_angle(self, byte1, byte2, byte3, is_heading=False):
        sign = -1 if (byte1 & 0x10) else 1  # Определение знака
        
        degrees = ((byte1 & 0x0F) * 100) + byte2 # Извлечение градусов и сотых долей
        hundredths = byte3 / 100.0

        angle = sign * (degrees + hundredths)

        # Для Heading знак не учитывается
        return abs(angle) if is_heading else angle

    def main(self):
        self.request_data()
        # time.sleep(0.1)  # Задержка перед следующим запросом


if __name__ == '__main__':
    rospy.init_node('rion_node')

    compas = Rion()
    while not rospy.is_shutdown():
        compas.main()