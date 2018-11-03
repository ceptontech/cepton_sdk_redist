function open_replay(capture_path)
    if cepton_sdk.capture_replay.is_open()
        cepton_sdk.capture_replay.close();
    end
    cepton_sdk.capture_replay.open(capture_path);

    cepton_sdk.capture_replay.resume_blocking(10);
    cepton_sdk.capture_replay.seek(0);
end