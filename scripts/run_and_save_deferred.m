function run_and_save_deferred(model, phase)
% Called by the StartFcn / StopFcn callbacks set by autosave_setup.
% Do not call it by hand: use run_and_save.

    % run started by run_and_save -> it already saves by itself
    if isappdata(0, 'run_and_save_active') && getappdata(0, 'run_and_save_active')
        return
    end

    names = tw_names(model);
    out_name = get_param(model, 'ReturnWorkspaceOutputsName');

    switch phase
        case 'start'
            % clear old data so the stop phase only sees the data of this run
            vars = [{out_name}, names];
            evalin('base', ['clear ' strjoin(vars, ' ')]);

        case 'stop'
            t = timer('StartDelay', 0.5, 'Period', 0.5, 'TasksToExecute', 40, ...
                      'ExecutionMode', 'fixedSpacing', 'Name', ['autosave_' model]);
            t.TimerFcn = @(tm, ~) try_save(tm, model, out_name, names);
            t.StopFcn  = @(tm, ~) delete(tm);
            start(t);
    end
end

function try_save(tm, model, out_name, names)
    % waits (max ~20 s) until the data of the run are in the workspace
    single_out = strcmp(get_param(model, 'ReturnWorkspaceOutputs'), 'on');
    if single_out
        ready = evalin('base', sprintf('exist(''%s'', ''var'')', out_name));
    else
        ready = any(cellfun(@(v) evalin('base', sprintf('exist(''%s'', ''var'')', v)), names));
    end
    if strcmp(get_param(model, 'SimulationStatus'), 'stopped') && ready
        stop(tm);
        try
            run_and_save(model, false);
        catch err
            warning('Autosave of %s failed: %s', model, err.message);
        end
    elseif tm.TasksExecuted >= tm.TasksToExecute
        warning('Autosave of %s: no data found after the run, nothing saved.', model);
    end
end

function names = tw_names(model)
    blk = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
                      'BlockType', 'ToWorkspace');
    names = get_param(blk, 'VariableName');
    if ischar(names), names = {names}; end
    names = unique(names(:)).';
end
