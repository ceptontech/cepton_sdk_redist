function callback_id = listen_frames(varargin)
    callback_id = cepton_sdk.internal.frames_callback().listen(varargin{:});
end