function result = get_frames_listener()
    persistent obj
    if isempty(obj)
        obj = cepton_sdk.internal.FramesListener();
    end
    result = obj;
end
