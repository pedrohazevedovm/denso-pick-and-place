import subprocess
import os
from time import sleep

# Dict with coordinates of camera button to take picture
models_specification = {'samsung_a34': {'x': 365, 'y': 1358, 'width': 78.1},
                        'samsung_a04e': {'x': 365, 'y': 1358, 'width': 75.9},
                        'moto_g32': {'x': 540, 'y': 2090, 'width': 76.4},
                        'virutal_device': {'x': 365, 'y': 1358, 'width': 78.1}}


class Device:
    def __init__(self, ip_port: str, model) -> None:
        self.__ip_port = ip_port
        self.model = model
        self.__camera__pos_dict = models_specification[self.model] if self.__is_model_camera_button_pos_mapped() else {}
        self.__connected = False
        self.width = models_specification[self.model]['width'] if self.__is_model_camera_button_pos_mapped() else 0

    @staticmethod
    def __start_adb() -> None:
        """
        Start the ADB server.
        """
        subprocess.run('adb start-server')

    def __is_model_camera_button_pos_mapped(self) -> bool:
        """
        Check if the device's button camera is in the 'camera_button_dict'.
        :return: bool
        """
        return True if self.model in models_specification else False

    def connect(self) -> None:
        """
        Connect the device to the ADB server.
        """
        self.__start_adb()
        sleep(1)
        subprocess.run(f'adb connect {self.__ip_port}')
        self.__connected = True

    def disconnect(self) -> None:
        """
        Disconnect the device from the ADB server.
        """
        subprocess.run(f'adb disconnect {self.__ip_port}')
        self.__connected = False

    def __open_camera(self) -> None:
        """
        Open the camera.
        """
        subprocess.run(f'adb -s {self.__ip_port} shell "input keyevent 27"')
        sleep(2)

    def take_picture(self) -> None:
        """
        Take a picture.
        """
        self.__open_camera()
        if not self.__camera__pos_dict:
            subprocess.run(f'adb -s {self.__ip_port} shell "input keyevent 27"')
        else:
            subprocess.run(f'adb -s {self.__ip_port} shell input tap {self.__camera__pos_dict["x"]} '
                           f'{self.__camera__pos_dict["y"]}')

    def __get_last_image(self) -> str:
        """
        Get the last image of the device gallery.
        :return: str with the name of the last image of the device.
        """
        sleep(3)
        images_list = subprocess.run(f'adb -s {self.__ip_port} shell ls /sdcard/DCIM/Camera', capture_output=True,
                                     text=True)
        gallery_items = images_list.stdout.split()
        image_list = []
        for image in gallery_items:
            if image[-4::1] == '.jpg':
                image_list.append(image)
        return image_list[-1]

    def save_photo(self) -> None:
        """
        Get the picture from device and save in PC.
        """
        last_image = self.__get_last_image()
        current_directory = os.getcwd()
        image_local_path = os.path.join(current_directory, r'images')
        if not os.path.exists(image_local_path):
            os.mkdir(image_local_path)
        subprocess.run(f'adb -s {self.__ip_port} pull /sdcard/DCIM/Camera/{last_image} {image_local_path}')

    def return_home(self) -> None:
        """
        Return the device screen to home.
        """
        subprocess.run(f'adb -s {self.__ip_port} shell input keyevent 3')

    def clear_gallery(self) -> None:
        """
        Delete all pictures from gallery.
        """
        subprocess.run(f'adb -s {self.__ip_port} shell rm /sdcard/DCIM/Camera/*')


if __name__ == '__main__':
    device = Device('192.168.158.230:38827', 'moto_g32')
    device.connect()
    device.take_picture()
    sleep(1)
    device.save_photo()
    device.clear_gallery()
    device.return_home()
    device.disconnect()
