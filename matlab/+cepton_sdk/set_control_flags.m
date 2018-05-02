function set_control_flags(mask, control_flags)
    mask = uint32(mask);
    control_flags = uint32(control_flags);
    error_code = ...
        cepton_sdk.c.call('cepton_sdk_set_control_flags', mask, control_flags);
    cepton_sdk.c.check_error_code(error_code);
end
