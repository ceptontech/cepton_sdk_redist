function result = is_open()
    result = cepton_sdk.c.call('cepton_sdk_capture_replay_is_open');
    result = logical(result);
end
