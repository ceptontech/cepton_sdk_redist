function open(path)
    cepton_sdk.capture_replay.close()
    error_code = cepton_sdk.c.call('cepton_sdk_capture_replay_open', path);
    cepton_sdk.c.check_error_code(error_code);
end
