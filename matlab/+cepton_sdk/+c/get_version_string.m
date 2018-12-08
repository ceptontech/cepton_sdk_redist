function result = get_version_string()
    result = cepton_sdk.c.call('cepton_sdk_get_version_string');
end