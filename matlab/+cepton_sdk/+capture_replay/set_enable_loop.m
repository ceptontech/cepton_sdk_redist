function set_enable_loop(enable_loop)
    cepton_sdk.c.call_and_check('cepton_sdk_capture_replay_set_enable_loop', enable_loop);
end
