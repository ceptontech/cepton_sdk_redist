function sensor_info = convert_sensor_information(c_sensor_info)
    sensor_info.handle = uint64(c_sensor_info.handle);
    sensor_info.serial_number = uint64(c_sensor_info.serial_number);
    sensor_info.model_name = fix_c_string(c_sensor_info.model_name);
    sensor_info.model = cepton_sdk.SensorModel(c_sensor_info.model);
    sensor_info.firmware_version = ...
        fix_c_string(c_sensor_info.firmware_version);

    sensor_info.measurement_period = double(c_sensor_info.measurement_period);

    sensor_info.ptp_ts = uint64(c_sensor_info.ptp_ts);

    sensor_info.gps_ts_year = uint8(c_sensor_info.gps_ts_year);
    sensor_info.gps_ts_month = uint8(c_sensor_info.gps_ts_month);
    sensor_info.gps_ts_day = uint8(c_sensor_info.gps_ts_day);
    sensor_info.gps_ts_hour = uint8(c_sensor_info.gps_ts_hour);
    sensor_info.gps_ts_min = uint8(c_sensor_info.gps_ts_min);
    sensor_info.gps_ts_sec = uint8(c_sensor_info.gps_ts_sec);

    sensor_info.return_count = uint8(c_sensor_info.return_count);
    sensor_info.segment_count = uint8(c_sensor_info.segment_count);

    flags = cepton_sdk.common.unpack_bits(c_sensor_info.flags, 32);;
    sensor_info.is_mocked = flags(1);
    sensor_info.is_pps_connected = flags(2);
    sensor_info.is_nmea_connected = flags(3);
    sensor_info.is_calibrated = flags(4);
end

function str = fix_c_string(c_str)
    str = native2unicode(c_str);
    str = strrep(str, char(0), ' ');
    str = strip(str, 'right');
end
