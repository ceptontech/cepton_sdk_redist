function result = get_error_code_name(error_code)
    result = cepton_sdk.c.call('cepton_get_error_code_name', error_code);
end
