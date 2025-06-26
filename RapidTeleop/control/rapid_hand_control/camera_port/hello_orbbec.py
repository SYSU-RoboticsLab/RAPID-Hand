from .camera_sdk.pyorbbecsdk import get_version, Context


def main():
    print("Hello Orbbec!")
    
    # Print SDK version
    sdk_version = get_version()
    print(f"SDK version: {sdk_version}")

    # Create device context and query devices
    context = Context()
    device_list = context.query_devices()
    device_count = device_list.get_count()

    if device_count == 0:
        print("No Orbbec device connected.")
        return

    # Access first available device
    device = device_list.get_device_by_index(0)
    device_info = device.get_device_info()
    print(f"Device Info:\n{device_info}")

    # List available sensors
    sensor_list = device.get_sensor_list()
    print("Available Sensors:")
    for i in range(sensor_list.get_count()):
        sensor = sensor_list.get_sensor_by_index(i)
        print(f"  - Sensor Type: {sensor.get_type()}")

    # Retrieve calibration parameters
    camera_param_list = device.get_calibration_camera_param_list()
    num_params = len(camera_param_list)

    print(f"Number of Camera Calibration Profiles: {num_params}")
    for i in range(num_params):
        camera_param = camera_param_list.get_camera_param(i)
        print(f"  - Profile #{i+1}: {camera_param}")


if __name__ == "__main__":
    main()
