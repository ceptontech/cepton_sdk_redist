function deinitialize()
    error_code = cepton_sdk.c.call('cepton_sdk_deinitialize');
    cepton_sdk.c.check_error_code(error_code);
end
