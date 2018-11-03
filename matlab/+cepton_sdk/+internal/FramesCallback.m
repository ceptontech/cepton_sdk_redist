classdef FramesCallback < handle

properties (Access = private)
    i_callback = uint64(0);
    callbacks = containers.Map('KeyType', 'uint64', 'ValueType', 'any');
end

methods
    function callback_id = listen(self, callback, callback_id_tmp)
        if nargin >= 3
            callback_id = callback_id_tmp;
        else
            callback_id = self.i_callback;
            self.i_callback = self.i_callback + 1;
        end
        assert(~isKey(self.callbacks, callback_id));
        self.callbacks(callback_id) = callback;
    end

    function unlisten(self, callback_id)
        assert(isKey(self.callbacks, callback_id));
        remove(self.callbacks, callback_id);
    end

    function update(self)
        while cepton_sdk.c.call('cepton_sdk_matlab_has_image_points')
            [sensor_handle, n_points] = ...
                cepton_sdk.c.call_and_check('cepton_sdk_matlab_get_image_points', 0, 0);

            sensor_info = ...
                cepton_sdk.get_sensor_information_by_handle(sensor_handle);

            a = zeros([n_points, 1]);
            [timestamps_usec, image_x, distances, image_z, intensities, return_types, flags_tmp] = ...
                cepton_sdk.c.call_and_check('cepton_sdk_matlab_get_image_points_data', a, a, a, a, a, a, a);
            flags = cepton_sdk.common.unpack_bits(flags_tmp, 8);

            points = cepton_sdk.Points(n_points);
            points.timestamps_usec(:) = timestamps_usec;
            points.distances(:) = distances;
            points.image_positions(:, 1) = image_x;
            points.image_positions(:, 2) = image_z;
            points.intensities(:) = intensities;
            points.return_types(:) = return_types;
            points.valid(:) = flags(:, 1);
            points.saturated(:) = flags(:, 2);

            points.positions = cepton_sdk.internal.convert_image_points_to_points(points.image_positions, points.distances);
            
            for key = keys(self.callbacks)
                callback = self.callbacks(key{1});
                callback(sensor_info, points);
            end
        end
    end
end

end
