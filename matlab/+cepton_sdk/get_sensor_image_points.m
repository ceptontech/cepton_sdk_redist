function image_points = get_sensor_image_points(serial_number, varargin)
    %image_points = GET_SENSOR_IMAGE_POINTS()
    %
    %   Returns next frame of image points for specified sensor.
    %
    %   Args:
    %       serial_number: Sensor serial number.

    %   Returns:
    %       Image points.
    default_args = struct();
    default_args.timeout = cepton_sdk.common.None;
    args = cepton_sdk.common.parse_args(default_args, varargin{:});

    persistent listeners_dict
    if isempty(listeners_dict)
        listeners_dict = ...
            containers.Map('KeyType', 'uint64', 'ValueType', 'any');
    end
    if ~listeners_dict.isKey(serial_number)
        listeners_dict(serial_number) = ...
            cepton_sdk.internal.SensorPointsListener(serial_number);
    end
    listener = listeners_dict(serial_number);

    if cepton_sdk.is_live()
        cepton_sdk.internal.clear_cache();
    end
    options = struct();
    options.timeout = args.timeout;
    cepton_sdk.internal.wait_on_func(@() listener.has_points(), options);
    image_points = listener.get_points();
end
