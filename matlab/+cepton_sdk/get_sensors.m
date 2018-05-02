function sensors_dict = get_sensors()
%sensors_dict = GET_SENSORS()
%
%   Returns attached sensors.
%
%   Returns:
%       Dictionary of sensors, indexed by serial number.
    n_sensors = cepton_sdk.get_n_sensors();

    sensors_dict = containers.Map('KeyType', 'uint64', 'ValueType', 'any');
    for i_sensor = 1:n_sensors
        sensor = cepton_sdk.Sensor.create_by_index(i_sensor);
        sensors_dict(sensor.serial_number) = sensor;
    end
end
