function result = image_frames_callback()
    persistent obj
    if isempty(obj)
        obj = cepton_sdk.internal.ImageFramesCallback();
    end
    result = obj;
end
