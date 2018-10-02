function close()
    cepton_sdk.c.call_and_check('cepton_sdk_capture_replay_close');
    cepton_sdk.internal.clear_cache();
end
