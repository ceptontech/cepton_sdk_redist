classdef Points < handle
%3D points array.

properties
    timestamps % time since epoch [microseconds]
    positions % x, y, z cartesian coordinates
    intensities % 0-1 scaled intensity
    return_numbers
    valid
end

properties (SetAccess = protected)
    n
end

methods
    function self = Points(n)
        self.n = n;
        self.timestamps = zeros([n, 1]);
        self.positions = zeros([n, 2]);
        self.intensities = zeros([n, 1]);
        self.return_numbers = zeros([n, 1], 'uint8');
        self.valid = zeros([n, 1], 'logical');
    end

    function points = select(self, indices)
        timestamps = self.timestamps(indices);
        n = numel(timestamps);
        points = cepton_sdk.Points(n);
        points.timestamps(:) = timestamps;
        points.positions(:, :) = self.positions(indices, :);
        points.intensities(:) = self.intensities(indices);
        points.return_numbers(:) = self.return_numbers(indices);
        points.valid(:) = self.valid(indices);
    end

    function put(self, indices, other)
        self.timestamps(indices) = other.timestamps;
        self.positions(indices, :) = other.positions;
        self.intensities(indices) = other.intensities;
        self.return_numbers(indices) = other.return_numbers;
        self.valid(indices) = other.valid;
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
        points.positions(:, :) = vertcat(points_list.positions);
        points.intensities(:) = vertcat(points_list.intensities);
        points.return_numbers(:) = vertcat(points_list.return_numbers);
        points.valid(:) = vertcat(points_list.valid);
    end
end

end
