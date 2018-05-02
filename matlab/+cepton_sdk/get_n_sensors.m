function result = get_n_sensors()
    result = cepton_sdk.c.call('cepton_sdk_get_n_sensors');
end
