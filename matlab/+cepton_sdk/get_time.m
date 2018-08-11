function result = get_time()
    if cepton_sdk.is_live()
        result = cepton_sdk.get_timestamp();
    else
        result = cepton_sdk.capture_replay.get_time();
    end
end
