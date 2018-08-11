function sensor_info = get_sensor_information_by_handle(sensor_handle)
    sensor_info = cepton_sdk.c.call_and_check('cepton_sdk_matlab_get_sensor_information', sensor_handle, struct());
    sensor_info = cepton_sdk.c.convert_sensor_information(sensor_info);
end
