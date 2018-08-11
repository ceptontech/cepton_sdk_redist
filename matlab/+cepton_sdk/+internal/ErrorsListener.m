classdef ErrorsListener

methods
    function update(self)
        while cepton_sdk.c.call('cepton_sdk_matlab_has_error')
            [handle, error_code, error_msg_size] = ...
                cepton_sdk.c.call_and_check('cepton_sdk_matlab_get_error', 0, 0, 0);

            options = struct();
            options.warning = true;
            cepton_sdk.c.check_error_code(error_code, options);
        end
    end
end

end
