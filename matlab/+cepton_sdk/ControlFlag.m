classdef ControlFlag < uint32
    enumeration
        DISABLE_NETWORK (2)
        DISABLE_IMAGE_CLIP (4)
        DISABLE_DISTANCE_CLIP (8)
        ENABLE_MULTIPLE_RETURNS (16)
        ENABLE_STRAY_FILTER (32)
        HOST_TIMESTAMPS (64)
        ENABLE_CROSSTALK_FILTER (128)
    end
end
