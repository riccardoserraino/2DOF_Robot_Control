function [x, y, vx, vy, ax, ay] = traj_select(t, id)
% TRAJ_SELECT  One entry point for all the tip paths: picks the path number "id".
%
% SIMULINK: one MATLAB Function block for every path:
%     function [x, y, vx, vy, ax, ay] = fcn(t, traj_id)
%     [x, y, vx, vy, ax, ay] = traj_select(t, traj_id);
%   t from a Clock, traj_id from a Constant block reading the workspace variable traj_id.
%   The block CALLS the files in scripts/trajectory, so the model and check_trajectory always
%   use the same code. Simulink compiles every file listed here: they must all exist.
% STOP TIME of the model: traj_select(-1, traj_id) = length of the chosen path.
%
% ADD A PATH: write its file (same 6 outputs, same t < 0 query), then add one "case" below.

switch id
    case 1, [x, y, vx, vy, ax, ay] = square_single(t);      % square, one speed
    case 2, [x, y, vx, vy, ax, ay] = square_speeds(t);      % square, several speed levels
    case 3, [x, y, vx, vy, ax, ay] = circle_single(t);      % circle, one speed
    case 4, [x, y, vx, vy, ax, ay] = triangle_single(t);    % triangle, one speed
    case 5, [x, y, vx, vy, ax, ay] = star_single(t);        % star, one speed
    case 6, [x, y, vx, vy, ax, ay] = rhombus_single(t);     % rhombus, one speed
    case 7, [x, y, vx, vy, ax, ay] = traj_sequence(t);      % benchmark: circle, square,
                                                             % rhombus at 3 speeds each
    otherwise                                                % unknown id: stay at home
        x = 0.127;  y = 0.127;  vx = 0;  vy = 0;  ax = 0;  ay = 0;
end
end
