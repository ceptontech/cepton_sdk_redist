function set_control_flags(mask, control_flags)
    mask = uint32(mask);
    control_flags = uint32(control_flags);
    cepton_sdk.c.call_and_check('cepton_sdk_set_control_flags', mask, control_flags);
end
