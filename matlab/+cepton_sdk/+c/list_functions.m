function list_functions()
    cepton_sdk.c.load();
    libfunctions('cepton_sdk', '-full');
end
