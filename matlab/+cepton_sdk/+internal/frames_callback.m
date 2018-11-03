function result = frames_callback()
    persistent obj
    if isempty(obj)
        obj = cepton_sdk.internal.FramesCallback();
    end
    result = obj;
end
