function result = get_sensor_handle(sensor_serial_number)
    result = cepton_sdk.c.call_and_check('cepton_sdk_get_sensor_handle_by_serial_number', sensor_serial_number, 0);
end
