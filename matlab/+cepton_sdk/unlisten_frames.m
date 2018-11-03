function unlisten_frames(varargin)
    cepton_sdk.internal.frames_callback().unlisten(varargin{:});
end