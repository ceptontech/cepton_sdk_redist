function wait()
    t_length = 0.1;
    if cepton_sdk.capture_replay.is_open()
        cepton_sdk.capture_replay.resume_blocking(t_length);
    else
        pause(t_length);
    end
    cepton_sdk.internal.update();
end
