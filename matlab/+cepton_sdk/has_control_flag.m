function result = has_control_flag(flag)
    flag = uint32(flag);
    result = cepton_sdk.c.call('cepton_sdk_has_control_flag', flag);
    result = logical(result);
end
