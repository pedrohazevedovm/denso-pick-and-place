import subprocess
import os
from time import sleep

camera_button_coords = {'motog4': {'x': 400, 'y': 2090}}

class Device:
    def __init__(self, ip_port: str) -> None:
        self.__ip_port = ip_port
        # self.__camera_dict

    @staticmethod
    def __start_adb() -> None:
        subprocess.run('adb start-server')

    def connect(self) -> None:
        self.__start_adb()
        sleep(1)
        subprocess.run(f'adb connect {self.__ip_port}')

    def disconnect(self) -> None:
        subprocess.run(f'adb disconnect {self.__ip_port}')

    def __open_camera(self) -> None:
        subprocess.run(f'adb -s {self.__ip_port} shell "input keyevent 27"')
        sleep(2)

    def take_photo(self) -> None:
        self.__open_camera()
        subprocess.run(f'adb -s {self.__ip_port} shell "input keyevent 27"')

    def __get_last_image(self) -> str:
        sleep(3)
        images_list = subprocess.run(f'adb -s {self.__ip_port} ls /sdcard/DCIM/Camera', capture_output=True,
                                     text=True)
        images = images_list.stdout.split()
        print(images)
        image_list = []
        for image in images:
            if image[-4::1] == '.jpg':
                image_list.append(image)

        return image_list[-1]

    def save_photo(self) -> None:
        last_image = self.__get_last_image()
        current_directory = os.getcwd()
        image_local_path = os.path.join(current_directory, r'images')
        if not os.path.exists(image_local_path):
            os.mkdir(image_local_path)
        subprocess.run(f'adb -s {self.__ip_port} pull /sdcard/DCIM/Camera/{last_image} {image_local_path}')


if __name__ == '__main__':
    device = Device('192.168.158.247:33671')
    device.connect()
    device.take_photo()
    sleep(1)
    device.save_photo()
    device.disconnect()
