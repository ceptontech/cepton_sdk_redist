classdef ListenerBase < handle

properties (SetAccess = protected)
    frame_id
    frames_listener = cepton_sdk.internal.get_frames_listener()
end

properties (SetAccess = private)
    reset_id
end

methods
    function self = ListenerBase()
        self.frame_id = self.frames_listener.frame_id;
        self.reset_id = self.frames_listener.reset_id;
    end

    function reset(self)
        self.frame_id = self.frames_listener.frame_id;
        self.reset_id = self.frames_listener.reset_id;
    end
end

methods (Access = protected)
    function update(self)
        if self.reset_id ~= self.frames_listener.reset_id
            self.reset()
        end
    end
end

end
