classdef ImageFramesListener < handle

properties (Access = private)
    callback_id;
    points_dict;
end

methods
    function self = ImageFramesListener()
        self.points_dict = containers.Map('KeyType', 'uint64', 'ValueType', 'any');
        self.callback_id = cepton_sdk.internal.image_frames_callback().listen(@self.on_points);
    end

    function delete(self)
        try
            cepton_sdk.internal.image_frames_callback().unlisten(self.callback_id);
        catch
        end
    end

    function reset(self)
        self.points_dict = containers.Map('KeyType', 'uint64', 'ValueType', 'any');
    end

    function result = has_points(self)
        result = ~isempty(self.points_dict);
    end

    function image_points_dict = get_points(self, varargin)
        cepton_sdk.internal.wait_on_func(@self.has_points, varargin{:});
        image_points_dict = self.get_points_impl();
    end
end

methods (Access = private)
    function on_points(self, serial_number, image_points)
        points_list = {};
        if isKey(self.points_dict, serial_number)
            points_list = self.points_dict(serial_number);
        end
        points_list{end + 1} = image_points;
        self.points_dict(serial_number) = points_list;
    end

    function points_dict = get_points_impl(self)
        points_dict = self.points_dict;
        self.reset();
    end
end

end
