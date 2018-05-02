function result = get_position()
    result = cepton_sdk.c.call('cepton_sdk_capture_replay_get_position');
end
