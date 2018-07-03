classdef ImagePoints < handle
%Image points array.

properties
    timestamps % time since epoch [microseconds]
    positions % x, z image coordinates
    distances % distance [meters]
    intensities % 0-1 scaled intensity
    return_numbers
    valid
end

properties (SetAccess = protected)
    n
end

methods
    function self = ImagePoints(n)
        self.n = n;
        self.timestamps = zeros([n, 1]);
        self.positions = zeros([n, 2]);
        self.distances = zeros([n, 1]);
        self.intensities = zeros([n, 1]);
        self.return_numbers = zeros([n, 1], 'uint8');
        self.valid = zeros([n, 1], 'logical');
    end

    function points = to_points(self)
        points = cepton_sdk.Points(self.n);
        points.timestamps(:) = self.timestamps;
        points.intensities(:) = self.intensities;
        points.return_numbers(:) = self.return_numbers;
        points.valid(:) = self.valid;

        focal_length_squared = 1;
        hypotenuse_small = sqrt(sum(self.positions.^2, 2) + focal_length_squared);
        ratio = self.distances ./ hypotenuse_small;
        points.positions(:, 1) = -self.positions(:, 1) .* ratio;
        points.positions(:, 2) = ratio;
        points.positions(:, 3) = -self.positions(:, 2) .* ratio;
    end

    function image_points = select(self, indices)
        n = nnz(indices);
        image_points = cepton_sdk.ImagePoints(n);
        image_points.timestamps(:) = self.timestamps(indices);
        image_points.positions(:, :) = self.positions(indices, :);
        image_points.distances(:) = self.distances(indices);
        image_points.intensities(:) = self.intensities(indices);
        image_points.return_numbers(:) = self.return_numbers(indices);
        image_points.valid(:) = self.valid(indices);
    end

    function put(self, indices, other)
        self.timestamps(indices) = other.timestamps;
        self.positions(indices, :) = other.positions;
        self.distances(indices) = other.distances;
        self.intensities(indices) = other.intensities;
        self.return_numbers(indices) = other.return_numbers;
        self.valid(indices) = other.valid;
    end

    function n = numel(self)
        n = self.n;
    end

    function tf = isempty(self)
        tf = self.n == 0;
    end
end

methods (Static)
    function image_points = combine(varargin)
        if nargin == 0
            image_points = cepton_sdk.ImagePoints(0);
            return
        end

        image_points_list = [varargin{:}];
        timestamps = vertcat(image_points_list.timestamps);
        n = numel(timestamps);
        image_points = cepton_sdk.ImagePoints(n);
        image_points.timestamps(:) = timestamps;
        image_points.positions(:, :) = vertcat(image_points_list.positions);
        image_points.distances(:) = vertcat(image_points_list.distances);
        image_points.intensities(:) = vertcat(image_points_list.intensities);
        image_points.return_numbers(:) = vertcat(image_points_list.return_numbers);
        image_points.valid(:) = vertcat(image_points_list.valid);
    end
end

end