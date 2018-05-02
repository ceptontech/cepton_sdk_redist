function sensor_info = get_sensor_information(sensor_serial_number)
    sensor_handle = cepton_sdk.get_sensor_handle(sensor_serial_number);
    sensor_info = cepton_sdk.get_sensor_information_by_handle(sensor_handle);
end
