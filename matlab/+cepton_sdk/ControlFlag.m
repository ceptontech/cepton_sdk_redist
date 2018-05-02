classdef ControlFlag < uint32
    enumeration
        DISABLE_NETWORK (2)
        DISABLE_IMAGE_CLIP (4)
        DISABLE_DISTANCE_CLIP (8)
        ENABLE_MULTIPLE_RETURNS (16)
    end
end
