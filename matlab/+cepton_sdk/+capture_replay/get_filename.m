function result = get_filename()
    result = cepton_sdk.c.call('cepton_sdk_capture_replay_get_filename');
end
