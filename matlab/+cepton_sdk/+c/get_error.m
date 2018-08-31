function result = get_error()
    error_msg = libpointer('stringPtrPtr', {''});
    error_code = cepton_sdk.c.call('cepton_sdk_get_error', error_msg);
    result = cepton_sdk.c.C_Error(error_code, error_msg.value{1});
end