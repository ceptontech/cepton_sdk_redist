function clear_cache()
    cepton_sdk.c.call('cepton_sdk_clear_cache');
    cepton_sdk.internal.clear_cache();
end
