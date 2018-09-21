classdef SensorImageFramesListener < handle

properties (SetAccess = private)
    serial_number;
end

properties (Access = private)
    callback_id;
    points_list;
    
end

methods
    function self = SensorImageFramesListener(serial_number)
        self.serial_number = serial_number;
        self.points_list = {};
        self.callback_id = cepton_sdk.internal.image_frames_callback().listen(@self.on_points);
    end

    function delete(self)
        try
            cepton_sdk.internal.image_frames_callback().unlisten(self.callback_id);
        catch
        end
    end

    function reset(self)
        self.points_list = {};
    end

    function result = has_points(self)
        result = ~isempty(self.points_list);
    end

    function image_points_list = get_points(self, varargin)
        cepton_sdk.internal.wait_on_func(@self.has_points, varargin{:});
        image_points_list = self.get_points_impl();
    end
end

methods (Access = private)
    function on_points(self, sensor_info, image_points)
        if sensor_info.serial_number ~= self.serial_number
            return;
        end
        self.points_list{end + 1} = image_points;
    end

    function points_list = get_points_impl(self)
        points_list = self.points_list;
        self.reset();
    end
end

end
