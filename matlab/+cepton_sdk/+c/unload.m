function unload()
    if ~libisloaded('cepton_sdk')
        return;
    end

    unloadlibrary('cepton_sdk');
end
