function sensor_info = get_sensor_information_by_handle(sensor_handle)
    [error_code, sensor_info] = ...
        cepton_sdk.c.call('cepton_sdk_matlab_get_sensor_information', ...
            sensor_handle, struct());
    cepton_sdk.c.check_error_code(error_code);
    sensor_info = cepton_sdk.c.convert_sensor_information(sensor_info);
end
