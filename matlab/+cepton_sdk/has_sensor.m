function result = has_sensor(sensor_serial_number)
    try
        cepton_sdk.get_sensor_handle(sensor_serial_number);
    except
        result = false;
        return
    end
    result = true;
end
