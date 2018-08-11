classdef SensorError < handle

properties (SetAccess = protected)
    code = 0;
    msg = '';
end

methods
    function self = SensorError(code, msg)
        if nargin >= 1
            self.code = code;
        end
        if nargin >= 2
            self.msg = msg;
        else
            
        end
    end

    function result = logical(self) 
        result = (self.code ~= 0);
    end
    
    function result = not(self)
        result = (self.code == 0);
    end

    function result = MException(self)
        id = sprintf('cepton_sdk_SensorError:%s', self.name);
        result = MException(id, self.msg);
    end

    function result = name(self)
        result = cepton_sdk.c.get_error_code_name(self.code);
    end

    function result = is_error(self)
        result = cepton_sdk.c.is_error_code(self.code);
    end

    function result = is_fault(self)
        result = cepton_sdk.c.is_fault_code(self.code);
    end
end

end