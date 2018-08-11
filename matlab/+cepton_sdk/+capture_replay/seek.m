function seek(position)
    cepton_sdk.c.call_and_check('cepton_sdk_capture_replay_seek', position);
end
