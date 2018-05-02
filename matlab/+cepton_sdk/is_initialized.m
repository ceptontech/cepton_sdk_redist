function result = is_initialized()
    result = cepton_sdk.c.call('cepton_sdk_is_initialized');
    result = logical(result);
end
