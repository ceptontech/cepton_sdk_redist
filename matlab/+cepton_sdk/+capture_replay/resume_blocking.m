function resume_blocking(t_length)
    cepton_sdk.c.call_and_check('cepton_sdk_capture_replay_resume_blocking', t_length);
end
