classdef PointsListener < cepton_sdk.internal.ListenerBase

methods
    function result = has_points(self)
        self.update();
        options = struct();
        options.frame_id_lb = self.frame_id;
        result = self.frames_listener.has_frames(options);
    end

    function points_dict = get_points(self)
        self.update();
        points_dict = ...
            containers.Map('KeyType', 'uint64', 'ValueType', 'any');

        options = struct();
        options.frame_id_lb = self.frame_id;
        frames = self.frames_listener.get_frames(options);
        if isempty(frames)
            return
        end

        self.frame_id = frames{end}.id + 1;

        for i_frame = 1:numel(frames)
            frame = frames{i_frame};
            if points_dict.isKey(frame.sensor_serial_number)
                points_list = points_dict(frame.sensor_serial_number);
                points_list{end + 1} = frame.points;
            else
                points_list = {};
                points_list{1} = frame.points;
            end
            points_dict(frame.sensor_serial_number) = points_list;
        end
    end
end

end
