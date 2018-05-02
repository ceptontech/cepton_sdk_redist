function result = get_enable_loop()
    result = ...
        cepton_sdk.c.call('cepton_sdk_capture_replay_get_enable_loop');
    result = logical(result);
end
