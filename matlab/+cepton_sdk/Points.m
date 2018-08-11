classdef Points < handle
%3D points array.

properties
    timestamps
    timestamps_usec
    positions
    intensities
    return_types
    valid
    saturated
end

properties (SetAccess = protected)
    n
end

methods
    function self = Points(n)
        self.n = n;
        self.timestamps = zeros([n, 1]);
        self.timestamps_usec = zeros([n, 1], 'int64');
        self.positions = zeros([n, 2]);
        self.intensities = zeros([n, 1]);
        self.return_types = zeros([n, 1], 'uint8');
        self.valid = zeros([n, 1], 'logical');
        self.saturated = zeros([n, 1], 'logical');
    end

    function points = select(self, indices)
        timestamps = self.timestamps(indices);
        n = numel(timestamps);
        points = cepton_sdk.Points(n);
        points.timestamps(:) = timestamps;
        points.timestamps_usec(:) = timestamps_usec;
        points.positions(:, :) = self.positions(indices, :);
        points.intensities(:) = self.intensities(indices);
        points.return_types(:) = self.return_types(indices);
        points.valid(:) = self.valid(indices);
        points.saturated(:) = self.saturated(indices);
    end

    function put(self, indices, other)
        self.timestamps(indices) = other.timestamps;
        self.timestamps_usec(indices) = other.timestamps_usec;
        self.positions(indices, :) = other.positions;
        self.intensities(indices) = other.intensities;
        self.return_types(indices) = other.return_types;
        self.valid(indices) = other.valid;
        self.saturated(indices) = other.saturated;
    end

    function n = numel(self)
        n = self.n;
    end

    function tf = isempty(self)
        disp(self.n)
        tf = self.n == 0;
    end
end

methods (Static)
    function points = combine(varargin)
        if nargin == 0
            points = cepton_sdk.Points(0);
            return
        end

        points_list = [varargin{:}];
        timestamps = vertcat(points_list.timestamps);
        n = numel(timestamps);
        points = cepton_sdk.Points(n);
        points.timestamps(:) = timestamps;
        points.timestamps_usec(:) = timestamps_usec;
        points.positions(:, :) = vertcat(points_list.positions);
        points.intensities(:) = vertcat(points_list.intensities);
        points.return_types(:) = vertcat(points_list.return_types);
        points.valid(:) = vertcat(points_list.valid);
        points.saturated(:) = vertcat(points_list.saturated);
    end
end

end
