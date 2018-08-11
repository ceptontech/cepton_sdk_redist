function open(path)
    cepton_sdk.capture_replay.close()
    cepton_sdk.c.call_and_check('cepton_sdk_capture_replay_open', path);
end
