%% FIX_CNC_MPC - applies the last corrections to slx/CNC_MPC.slx (run once)
% 1. replaces From9 / From12 (tags Input3165 / Input3166, no Goto) with From
%    blocks on Control3165 / Control3166, so the scopes show the voltage
% 2. renames the To Workspace variables to the names used by validate_controllers
% 3. connects Control3165 to the second input of the To Host File Mux
% The model can be open: the script edits it and saves it.

mdl  = 'CNC_MPC';
here = fileparts(mfilename('fullpath'));
open_system(fullfile(here, '..', '..', 'slx', [mdl '.slx']));

%% 1. From blocks without a Goto -> From on the control voltages
swap = {'From9',  '3165', 'Control3165';
        'From12', '3166', 'Control3166'};
for i = 1:size(swap, 1)
    old = [mdl '/' swap{i,1}];
    if getSimulinkBlockHandle(old) == -1
        fprintf('%s already removed\n', swap{i,1});  continue
    end
    pos = get_param(old, 'Position');
    lh  = get_param(old, 'LineHandles');
    if lh.Outport > 0, delete_line(lh.Outport); end
    delete_block(old);
    newname = ['From_u' swap{i,2}];
    add_block('simulink/Signal Routing/From', [mdl '/' newname], ...
              'GotoTag', swap{i,3}, 'Position', pos);
    add_line(mdl, [newname '/1'], [swap{i,2} '/2'], 'autorouting', 'on');
    fprintf('%s (%s) replaced by %s (%s) -> scope %s input 2\n', ...
            swap{i,1}, ['Input' swap{i,2}], newname, swap{i,3}, swap{i,2});
end

%% 2. To Workspace variable names
ren = {'To Workspace11', 'u_3165';
       'To Workspace12', 'u_3166';
       'To Workspace16', 'usat_3165';
       'To Workspace3',  'usat_3166';
       'To Workspace13', 'r_3165';
       'To Workspace4',  'r_3166';
       'To Workspace15', 'xc_des';
       'To Workspace14', 'yc_des'};
for i = 1:size(ren, 1)
    blk = [mdl '/' ren{i,1}];
    fprintf('%-15s %-8s -> %s\n', ren{i,1}, get_param(blk, 'VariableName'), ren{i,2});
    set_param(blk, 'VariableName', ren{i,2});
end

%% 3. Second input of the To Host File Mux
lh = get_param([mdl '/Mux'], 'LineHandles');
if numel(lh.Inport) >= 2 && lh.Inport(2) == -1
    pm = get_param([mdl '/From'], 'Position');                 % From feeding Mux input 1
    add_block('simulink/Signal Routing/From', [mdl '/From_mux3165'], ...
              'GotoTag', 'Control3165', 'Position', pm + [0 40 0 40]);
    add_line(mdl, 'From_mux3165/1', 'Mux/2', 'autorouting', 'on');
    fprintf('Mux input 2 <- Control3165\n');
end

%% Check: every From has a Goto
froms = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'From');
gotos = get_param(find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Goto'), 'GotoTag');
bad = froms(~ismember(get_param(froms, 'GotoTag'), gotos));
if isempty(bad), fprintf('All From blocks have a matching Goto.\n');
else, warning('From blocks without Goto: %s', strjoin(bad', ', '));
end

save_system(mdl);
fprintf('%s saved.\n', mdl);
