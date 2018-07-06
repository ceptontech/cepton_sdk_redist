function load()
    if libisloaded('cepton_sdk')
        return;
    end

    if ispc()
        os_name = 'win64';
        lib_name = 'cepton_sdk_matlab';
    else if isunix()
        if ismac()
            os_name = 'osx';
        else
            os_name = 'linux-x86_64';
        end
        lib_name = 'libcepton_sdk_matlab';
    else
        error('Platform not supported!');
    end

    package = what('cepton_sdk');
    package_dir = fileparts(package.path);
    include_dir = fullfile(package_dir, 'include');
    lib_dir = fullfile(package_dir, 'lib', cepton_sdk.c.get_os_name());
    
    warning('off');
    loadlibrary(fullfile(lib_dir, lib_name), ...
        fullfile(include_dir, 'cepton_sdk_matlab.h'), ...
        'alias', 'cepton_sdk', ...
        'includepath', include_dir, ...
        'addheader', 'cepton_sdk.h');
    warning('on');
end
