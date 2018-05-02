function result = errors_listener()
    persistent obj
    if isempty(obj)
        obj = cepton_sdk.internal.ErrorsListener();
    end
    result = obj;
end
