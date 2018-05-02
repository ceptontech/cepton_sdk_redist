function result = get_start_time()
    result = cepton_sdk.c.call('cepton_sdk_capture_replay_get_start_time');
    result = 1e-6 * double(result);
end
