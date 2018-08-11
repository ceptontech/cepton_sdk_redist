function sensor_info = get_sensor_information_by_index(sensor_index)
    sensor_info = cepton_sdk.c.call_and_check('cepton_sdk_matlab_get_sensor_information_by_index', sensor_index - 1, struct());
    sensor_info = cepton_sdk.c.convert_sensor_information(sensor_info);
end
