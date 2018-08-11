function unlisten_image_frames(varargin)
    cepton_sdk.internal.image_frames_callback().unlisten(varargin{:});
end