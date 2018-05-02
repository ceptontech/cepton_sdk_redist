function pause()
    error_code = cepton_sdk.c.call('cepton_sdk_capture_replay_pause');
    cepton_sdk.c.check_error_code(error_code);
end
