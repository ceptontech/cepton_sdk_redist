function result = get_control_flags()
    result = cepton_sdk.c.call('cepton_sdk_get_control_flags');
    result = uint32(result);
end
