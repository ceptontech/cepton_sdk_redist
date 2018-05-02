function clear_cache()
    frames_listener = cepton_sdk.internal.get_frames_listener();
    frames_listener.update();
    frames_listener.reset();
end
