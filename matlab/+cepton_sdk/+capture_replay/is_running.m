function result = is_running()
    result = cepton_sdk.c.call('cepton_sdk_capture_replay_is_running');
    result = logical(result);
end
