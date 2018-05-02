function result = get_lib_name()
    if isunix() | ismac()
        result = 'libcepton_sdk_matlab';
    elseif ispc()
        result = 'cepton_sdk_matlab';
    else
        error('Platform not supported!');
    end
end
