function thrust_tbl = import_motor_thrust_curve(file_path, n_samples, variation_pct)
% Import .ENG or .RSE thrust curves and generate Monte Carlo variants.

if nargin < 2 || isempty(n_samples), n_samples = 200; end
if nargin < 3 || isempty(variation_pct), variation_pct = 5; end

[~, ~, ext] = fileparts(file_path);
ext = lower(ext);

switch ext
    case '.eng'
        [t, f] = parse_eng(file_path);
    case '.rse'
        [t, f] = parse_rse(file_path);
    otherwise
        error('Unsupported motor file format: %s', ext);
end

scale = 1 + (variation_pct / 100) * randn(n_samples, 1);
scale = max(scale, 0.7);

rows = [];
sample_id = [];
for i = 1:n_samples
    fi = f * scale(i);
    rows = [rows; [t, fi]]; %#ok<AGROW>
    sample_id = [sample_id; i * ones(numel(t), 1)]; %#ok<AGROW>
end

thrust_tbl = table(sample_id, rows(:,1), rows(:,2), ...
    'VariableNames', {'sample_id', 'time_s', 'thrust_N'});

end

function [t, f] = parse_eng(file_path)
raw = fileread(file_path);
lines = splitlines(string(raw));
vals = [];
for i = 1:numel(lines)
    L = strtrim(lines(i));
    if L == "" || startsWith(L, ';')
        continue;
    end
    parts = split(L);
    if numel(parts) >= 2
        a = str2double(parts(1));
        b = str2double(parts(2));
        if ~isnan(a) && ~isnan(b)
            vals = [vals; a, b]; %#ok<AGROW>
        end
    end
end
if isempty(vals)
    error('No thrust samples parsed from ENG file.');
end
t = vals(:,1);
f = vals(:,2);
end

function [t, f] = parse_rse(file_path)
xml = xmlread(file_path);
pts = xml.getElementsByTagName('eng-data');
n = pts.getLength;
if n == 0
    error('No eng-data nodes found in RSE file.');
end

t = zeros(n,1); f = zeros(n,1);
for i = 0:n-1
    node = pts.item(i);
    t(i+1) = str2double(char(node.getAttribute('t')));
    f(i+1) = str2double(char(node.getAttribute('f')));
end
end
