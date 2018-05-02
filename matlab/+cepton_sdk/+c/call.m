function varargout = call(varargin)
    cepton_sdk.c.load();
    lib_name = cepton_sdk.c.get_lib_name();
    [varargout{1:nargout}] = calllib(lib_name, varargin{:});
end
