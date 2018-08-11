function plot_points(points)
    x = points.positions(:, 1);
    y = points.positions(:, 2);
    z = points.positions(:, 3);
    s = 3;
    c = [1, 1, 1];
    h = scatter3(x, y, z, s, c, 'filled');

    fig = gcf();
    ax = gca();

    % Set background color
    fig.Color = 'black';

    % Disable bounds clipping
    ax.Clipping = 'off';

    % Fix aspect ratio
    axis(ax, 'equal');
    axis(ax, 'vis3d');

    % Disable axes
    axis(ax, 'off');
    grid(ax, 'off');

    % Enable 3d rotation cursor
    rotate3d(ax, 'on');
end
