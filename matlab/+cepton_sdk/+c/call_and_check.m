function varargout = call_and_check(varargin)
    cepton_sdk.c.load();
    [error_code, varargout{1:nargout}] = calllib('cepton_sdk', varargin{:});
    cepton_sdk.c.check_error(cepton_sdk.c.get_error());
end
