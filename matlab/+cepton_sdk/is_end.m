function result = is_end()
    if cepton_sdk.capture_replay.is_open()
        if cepton_sdk.capture_replay.get_enable_loop()
            result = false;
        else
            result = cepton_sdk.capture_replay.is_end();
        end
    else
        result = false;
    end
end
