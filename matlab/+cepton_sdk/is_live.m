function result = is_live()
    result = ~cepton_sdk.capture_replay.is_open();
end
