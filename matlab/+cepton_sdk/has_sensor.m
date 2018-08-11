function result = has_sensor(serial_number)
    try
        cepton_sdk.get_sensor_handle(serial_number);
    except
        result = false;
        return
    end
    result = true;
end
