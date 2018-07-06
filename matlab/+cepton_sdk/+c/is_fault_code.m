function result = is_fault_code(error_code)
    result = cepton_sdk.c.call('cepton_is_fault_code', error_code);
    result = logical(result);
end