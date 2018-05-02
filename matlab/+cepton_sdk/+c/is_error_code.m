function result = is_error_code(error_code)
    result = cepton_sdk.c.call('cepton_sdk_is_error_code', error_code);
    result = logical(result);
end