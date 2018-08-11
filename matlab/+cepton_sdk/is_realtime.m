function result = is_realtime()
    result = cepton_sdk.is_live() | cepton_sdk.capture_replay.is_running();
end
