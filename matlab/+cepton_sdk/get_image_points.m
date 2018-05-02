function image_points_dict = get_image_points(varargin)
%image_points_dict = GET_IMAGE_POINTS()
%
%   Returns next frames of image points for all sensors.
%
%   Returns:
%       Dictionary of lists of frames, indexed by serial number.
    default_args = struct();
    default_args.timeout = cepton_sdk.common.None;
    args = cepton_sdk.common.parse_args(default_args, varargin{:});

    persistent listener
    if isempty(listener)
        listener = cepton_sdk.internal.PointsListener();
    end

    if cepton_sdk.is_live()
        cepton_sdk.internal.clear_cache();
    end
    options = struct();
    options.timeout = args.timeout;
    cepton_sdk.internal.wait_on_func(@() listener.has_points(), options);
    image_points_dict = listener.get_points();
end
