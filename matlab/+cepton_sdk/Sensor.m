classdef Sensor < handle

properties (SetAccess = protected)
    information
end

methods
    function self = Sensor(sensor_info)
        self.information = sensor_info;
    end

    function result = handle(self)
        result = self.information.handle;
    end

    function result = serial_number(self)
        result = self.information.serial_number;
    end

    function update(self)
        self.sensor_info = ...
            cepton_sdk.get_sensor_information_by_handle(self.handle);
    end

    function image_points = get_image_points(self, frame_length)
        image_points = ...
            cepton_sdk.api.get_sensor_image_points( ...
                self.serial_number, t_length);
    end
end

methods (Static)
    function obj = create(sensor_serial_number)
        sensor_info = cepton_sdk.get_sensor_information(sensor_serial_number);
        obj = cepton_sdk.Sensor(sensor_info);
    end

    function obj = create_by_index(sensor_index)
        sensor_info = ...
            cepton_sdk.get_sensor_information_by_index(sensor_index);
        obj = cepton_sdk.Sensor(sensor_info);
    end

    function obj = create_by_handle(sensor_handle)
        sensor_info = ...
            cepton_sdk.get_sensor_information_by_handle(sensor_handle);
        obj = cepton_sdk.Sensor(sensor_info);
    end
end

end
