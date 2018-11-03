function positions = convert_image_points_to_points(image_positions, distances)
    focal_length_squared = 1;
    hypotenuse_small = sqrt(sum(image_positions.^2, 2) + focal_length_squared);
    ratio = distances ./ hypotenuse_small;
    n = size(image_positions, 1);
    positions = zeros([n, 3]);
    positions(:, 1) = -image_positions(:, 1) .* ratio;
    positions(:, 2) = ratio;
    positions(:, 3) = -image_positions(:, 2) .* ratio;
end