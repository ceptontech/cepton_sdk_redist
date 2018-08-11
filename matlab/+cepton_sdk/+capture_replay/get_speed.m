function result = get_speed()
    result = cepton_sdk.c.call('cepton_sdk_capture_replay_get_speed');
    result = double(result);
end
