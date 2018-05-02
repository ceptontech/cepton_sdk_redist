function list_functions()
    cepton_sdk.c.load();
    lib_name = cepton_sdk.c.get_lib_name();
    libfunctions(lib_name, '-full');
end
