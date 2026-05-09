function out = s2r_prepare_output_dirs(cfg)
out.root = cfg.paths.output_root;
out.graphs = fullfile(out.root, 'graphs');
out.sheets = fullfile(out.root, 'sheets');
out.logs = fullfile(out.root, 'logs');

if ~exist(out.root, 'dir'), mkdir(out.root); end
if ~exist(out.graphs, 'dir'), mkdir(out.graphs); end
if ~exist(out.sheets, 'dir'), mkdir(out.sheets); end
if ~exist(out.logs, 'dir'), mkdir(out.logs); end

% Clear stale artifacts for deterministic reruns.
delete_if_exists(fullfile(out.graphs, '*.png'));
delete_if_exists(fullfile(out.sheets, '*.csv'));
delete_if_exists(fullfile(out.logs, '*.txt'));

end

function delete_if_exists(pattern)
files = dir(pattern);
for i = 1:numel(files)
    delete(fullfile(files(i).folder, files(i).name));
end
end
