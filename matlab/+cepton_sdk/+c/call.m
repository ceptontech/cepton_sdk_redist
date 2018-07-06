function varargout = call(varargin)
    cepton_sdk.c.load();
    [varargout{1:nargout}] = calllib('cepton_sdk', varargin{:});
end
