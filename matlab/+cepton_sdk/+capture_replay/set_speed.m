function set_speed(speed)
    cepton_sdk.c.call_and_check('cepton_sdk_capture_replay_set_speed', speed);
end
