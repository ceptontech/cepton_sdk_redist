classdef SensorPointsListener < cepton_sdk.internal.ListenerBase

properties (SetAccess = protected)
    sensor_serial_number
end

methods
    function self = SensorPointsListener(sensor_serial_number)
        self.sensor_serial_number = sensor_serial_number;
    end

    function result = has_points(self)
        self.update();
        options = struct();
        options.sensor_serial_number = self.sensor_serial_number;
        options.frame_id_lb = self.frame_id;
        result = self.frames_listener.has_frames(options);
    end

    function points = get_points(self)
        self.update();

        options = struct();
        options.frame_id_lb = self.frame_id;
        options.max_n_frames = 1;
        options.sensor_serial_number = self.sensor_serial_number;
        frames = self.frames_listener.get_frames(options);
        if isempty(frames)
            return
        end
        frame = frames{1};
        points = frame.points;
        self.frame_id = frame.id + 1;
    end

end

end
