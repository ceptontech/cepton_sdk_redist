function load()
    lib_name = cepton_sdk.c.get_lib_name();
    if ~libisloaded(lib_name)
        warning('off');
        loadlibrary(lib_name, 'cepton_sdk_matlab.h', 'addheader', 'cepton_sdk.h');
        warning('on');
    end
end
