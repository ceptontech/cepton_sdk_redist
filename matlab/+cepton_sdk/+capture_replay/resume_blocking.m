function resume_blocking(t_length)
    error_code = ...
        cepton_sdk.c.call('cepton_sdk_capture_replay_resume_blocking', t_length);
    cepton_sdk.c.check_error_code(error_code);
end
