function callback_id = listen_image_frames(varargin)
    callback_id = cepton_sdk.internal.image_frames_callback().listen(varargin{:});
end