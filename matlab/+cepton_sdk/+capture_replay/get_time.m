function result = get_time()
    result = cepton_sdk.capture_replay.get_start_time() + cepton_sdk.capture_replay.get_position();
end
