function tag = run_and_save(model, do_run)
% RUN_AND_SAVE  Runs a Simulink model (or collects the data of a real run)
% and saves every To Workspace signal as  <signal>_<model>_<yyyymmdd>_<HHMMSS>_traj<id>
% both in the base workspace and in <repo>/data/sim/<model>_<yyyymmdd>_<HHMMSS>_traj<id>.mat
% (simulation) or <repo>/data/lab/... (QUARC model in external mode = bench run).
% <id> = traj_id of the base workspace, the path the model ran (traj_select.m numbers);
% without traj_id the name ends after the time, as before.
%
%   run_and_save('Model_pp_obs')          simulation: runs sim() and saves
%   run_and_save('CNC_pp_luenberg')       QUARC model (external mode): it does NOT
%                                         run it, it saves the data of the run
%                                         you have just done on the bench
%   run_and_save('Model_pp_obs', false)   saves only, without running
%
% The file also contains, with their original names, the controller gains and
% plant parameters found in the base workspace (kp_xxxx, Kx_xxxx, A_obs_xxxx,
% k_xxxx_load, Vmax, ...) and run_tag: validate_controllers needs them.
%
% Example of created variables (traj_id = 7):
%   xc_des_Model_pp_obs_20261124_170512_7,  xc_sim_Model_pp_obs_20261124_170512_traj7, ...
% The returned tag is '<model>_<yyyymmdd>_<HHMMSS>_<traj_id>'. The file also holds traj_id.

    % char(): a "string" input (double quotes) would turn [model '_' stamp] into a string
    % ARRAY of 3 pieces, and a struct field name must be one piece of text
    [~, model] = fileparts(char(model));    % accepts also 'Model_pp_obs.slx'
    load_system(model);

    is_real = strcmpi(get_param(model, 'SimulationMode'), 'external');
    if nargin < 2, do_run = ~is_real; end

    % names of the To Workspace variables of the model (also inside subsystems)
    % MatchFilter: search every variant choice (newer MATLAB asks for it explicitly)
    blk = find_system(model, 'MatchFilter', @Simulink.match.allVariants, ...
                      'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                      'BlockType', 'ToWorkspace');
    names = get_param(blk, 'VariableName');
    if ischar(names), names = {names}; end
    names = unique(names);
    if isempty(names)
        error('%s has no To Workspace blocks.', model);
    end

    % ---- run or take the data already in the workspace ----
    if do_run
        fprintf('Running %s ...\n', model);
        setappdata(0, 'run_and_save_active', true);   % autosave callbacks: skip
        cleanup = onCleanup(@() setappdata(0, 'run_and_save_active', false));
        so = sim(model, 'ReturnWorkspaceOutputs', 'on');
    else
        % read the output name ONLY if single-output mode is on: newer MATLAB refuses to
        % read ReturnWorkspaceOutputsName when ReturnWorkspaceOutputs is off
        so = [];                            % default: variables directly in the workspace
        if strcmp(get_param(model, 'ReturnWorkspaceOutputs'), 'on')
            out_name = get_param(model, 'ReturnWorkspaceOutputsName');
            if evalin('base', sprintf('exist(''%s'', ''var'')', out_name))
                so = evalin('base', out_name);
            end
        end
    end

    % ---- tag and name length check ----
    stamp = datestr(now, 'yyyymmdd_HHMMSS');  %#ok<TNOW1,DATST>
    % trajectory of the run: traj_id of the base workspace (read by the trajectory block)
    traj_id = [];
    if evalin('base', 'exist(''traj_id'', ''var'')')
        traj_id = evalin('base', 'traj_id');
        stamp   = sprintf('%s_%d', stamp, traj_id);   % e.g. 20261124_170512_7
    end
    tag   = [model '_' stamp];
    lmax  = max(cellfun(@numel, names)) + 1 + numel(tag);
    if lmax > namelengthmax
        % MATLAB names are limited to 63 characters: shorten the model name
        keep = numel(model) - (lmax - namelengthmax);
        tag  = [model(1:keep) '_' stamp];
        warning('Names too long: model name shortened to "%s".', model(1:keep));
    end

    % ---- collect, rename, save ----
    S = struct();
    for k = 1:numel(names)
        v = names{k};
        val = [];
        if ~isempty(so) && any(strcmp(so.who, v))
            val = so.get(v);
        elseif evalin('base', sprintf('exist(''%s'', ''var'')', v))
            val = evalin('base', v);
        end
        if isempty(val)
            fprintf('  %-10s not found, skipped\n', v);
            continue
        end
        newname = [v '_' tag];
        S.(newname) = val;
        assignin('base', newname, val);
        fprintf('  %-10s -> %s\n', v, newname);
    end
    missing = setdiff(names, regexprep(fieldnames(S), ['_' tag '$'], ''));
    if ~isempty(missing) && ~isempty(fieldnames(S)) && isempty(so)
        % without a simulation output, the found signals come from the base workspace:
        % they may be LEFTOVERS of an older run, while the missing ones were never there
        warning(['%s not found while others were read from the base workspace: ' ...
                 'they may belong to an older run. Run the model (do_run = true) ' ...
                 'or check where the last simulation stored its outputs.'], strjoin(missing, ', '));
    end
    if isempty(fieldnames(S))
        error('No signal found: did the run finish and produce data?');
    end

    % ---- controller gains and plant parameters (original names, no tag) ----
    % needed by validate_controllers to rebuild the loop (Ms, Nyquist, ...)
    pat  = ['^((kp|ki|kd|Kx|Ki|Kaw|Kffv|Kffa|A_obs|B_obs)_\d{4}' ...
            '|(k|tau)_\d{4}_(load|noload|ff)|Vc_\d{4}(_ff)?|Vmax|N_pid)$'];
    vars = evalin('base', 'who');
    vars = vars(~cellfun(@isempty, regexp(vars, pat, 'once')));
    for k = 1:numel(vars)
        S.(vars{k}) = evalin('base', vars{k});
    end
    S.run_tag = tag;
    if ~isempty(traj_id), S.traj_id = traj_id; end         % path of the run
    fprintf('  + %d gains/parameters: %s\n', numel(vars), strjoin(vars(:)', ' '));

    % <repo>/data/sim, wherever this file is: go up until a folder with 'data'
    root = fileparts(mfilename('fullpath'));
    while ~isfolder(fullfile(root, 'data'))
        parent = fileparts(root);
        if strcmp(parent, root), error('Folder "data" not found above %s', mfilename('fullpath')); end
        root = parent;
    end
    if is_real, sub = 'lab'; else, sub = 'sim'; end   % bench (QUARC) or simulation
    folder = fullfile(root, 'data', sub);
    if ~isfolder(folder), mkdir(folder); end
    file = fullfile(folder, [tag '.mat']);
    save(file, '-struct', 'S');
    fprintf('Saved %d variables in %s\n', numel(fieldnames(S)), file);
end