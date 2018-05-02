classdef FramesListener < handle

properties (Constant)
    max_n_frames = 100
end

properties (SetAccess = private)
    reset_id = int64(1)
    frame_id = int64(1)
end

properties (Access = private)
    frames = {}
end

methods
    function reset(self)
        self.reset_id = self.reset_id + 1;
        self.frames = {};
    end

    function update(self)
        while cepton_sdk.c.call('cepton_sdk_matlab_has_image_points')
            [error_code, sensor_handle, n_points] = ...
                cepton_sdk.c.call('cepton_sdk_matlab_get_image_points', 0, 0);
            cepton_sdk.c.check_error_code(error_code);

            sensor_info = ...
                cepton_sdk.get_sensor_information_by_handle(sensor_handle);

            a = zeros([n_points, 1]);
            [error_code, timestamps, image_x, distances, image_z, intensities, return_numbers, valid] = ...
                cepton_sdk.c.call('cepton_sdk_matlab_get_image_points_data', a, a, a, a, a, a, a);
            cepton_sdk.c.check_error_code(error_code);

            image_points = cepton_sdk.ImagePoints(n_points);
            image_points.timestamps(:) = 1e-6 * double(timestamps);
            image_points.positions(:, 1) = image_x;
            image_points.distances(:) = distances;
            image_points.positions(:, 2) = image_z;
            image_points.intensities(:) = intensities;
            image_points.return_numbers(:) = return_numbers;
            image_points.valid(:) = valid;

            frame = struct();
            frame.id = self.frame_id;
            frame.sensor_serial_number = sensor_info.serial_number;
            frame.points = image_points;
            self.frame_id = self.frame_id + 1;
            self.frames{end + 1} = frame;
            while numel(self.frames) > self.max_n_frames
                self.frames(1) = [];
            end
        end
    end

    function is_valid = get_valid_frames(self, varargin)
        default_args = struct();
        default_args.frame_id_lb = cepton_sdk.common.None;
        default_args.max_n_frames = cepton_sdk.common.None;
        default_args.sensor_serial_number = cepton_sdk.common.None;
        args = cepton_sdk.common.parse_args(default_args, varargin{:});

        n_frames = numel(self.frames);
        n_valid_frames = 0;
        is_valid = zeros([n_frames, 1], 'logical');
        for i_frame = 1:n_frames
            frame = self.frames{i_frame};

            if ~cepton_sdk.common.is_none(args.max_n_frames)
                if n_valid_frames >= args.max_n_frames
                    break
                end
            end

            if ~cepton_sdk.common.is_none(args.frame_id_lb)
                if frame.id < args.frame_id_lb
                    continue
                end
            end

            if ~cepton_sdk.common.is_none(args.sensor_serial_number)
                if frame.sensor_serial_number ~= args.sensor_serial_number
                    continue
                end
            end

            is_valid(i_frame) = true;
        end
    end

    function frame_id = get_frame_id(self)
        frame_id = self.frame_id;
    end

    function result = has_frames(self, varargin)
        is_valid = self.get_valid_frames(varargin{:});
        result = any(is_valid);
    end

    function frames = get_frames(self, varargin)
        is_valid = self.get_valid_frames(varargin{:});
        indices = find(is_valid);
        frames = self.frames(indices);
    end
end

end
