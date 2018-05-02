function result = is_end()
    result = cepton_sdk.c.call('cepton_sdk_capture_replay_is_end');
    result = logical(result);
end
