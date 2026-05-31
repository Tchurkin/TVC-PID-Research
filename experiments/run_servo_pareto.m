% run_servo_pareto.m
% Map a representative set of commonly-used hobby/competition servos onto the
% phase diagram. Produces "for $X you can stabilize up to p_max" — the kind of
% concrete engineering table no prior TVC paper has published.
%
% Servo slew specs are RATED no-load speeds from manufacturer datasheets.
% Loaded slew under a TVC gimbal is typically 50-70% of rated, depending on
% torque margin. We use a conservative 0.6 derate, then divide by the linkage
% ratio (gimbal_deg per servo_deg). Default linkage = 1/4 (servo moves 4 deg
% to swing gimbal 1 deg) per project convention.

clear; clc;
here = fileparts(mfilename('fullpath'));

% --- Power-law parameters from run_scaling_law.m ---
%   slew_min(p) = a * p^alpha  with a=0.316, alpha=1.99 (R^2=0.75, theory: alpha=2)
% Invert: p_max(slew) = (slew/a)^(1/alpha)
fit = readtable(fullfile(here,'results','scaling_law_fit.csv'));
a     = fit.coef_a(1);
alpha = fit.exponent_alpha(1);
fprintf('Using fitted scaling law:  slew_min = %.3f * p^%.3f\n', a, alpha);
fprintf('Inverse:                    p_max(slew) = (slew/%.3f)^(1/%.3f)\n\n', a, alpha);

% --- Servo specs from public datasheets (no-load slew, msrp street price USD) ---
% Slew = 60/transit_time_per_60deg, deg/s, rated voltage in parens
servos = {
%   name                 slew_rated_dps  price_usd  voltage  note
    'TowerPro SG90',          600,        4,       4.8,   'micro hobby'
    'TowerPro MG90S',         600,        7,       4.8,   'metal-gear micro'
    'Hitec HS-422',           330,       12,       4.8,   'classic analog'
    'TowerPro MG996R',        333,        9,       6.0,   'common high-torque'
    'Savox SH-0255MG',        545,       35,       6.0,   'mini digital'
    'Hitec HS-5070MH',        545,       45,       7.4,   'HV digital'
    'Dynamixel XL330-M288',   480,       35,       5.0,   'serial bus, smart'
    'Hitec HS-7950TH',        545,      170,       7.4,   'high-end coreless'
    'KST X10 Mini',           750,       60,       7.4,   'high-perf mini'
    'BLS-HV70',               750,       30,       7.4,   'brushless HV (cheap)'
    'Savox SB-2290SG',        545,      230,       7.4,   'brushless competition'
};

servo_names   = servos(:,1);
slew_rated    = cell2mat(servos(:,2));
prices        = cell2mat(servos(:,3));

% --- Conversion to gimbal slew available to the controller ---
derate        = 0.6;     % loaded vs no-load
linkage_ratio = 4.0;     % servo_deg per gimbal_deg
gimbal_slew   = slew_rated * derate / linkage_ratio;

% --- Predicted max stable p_unstable per servo ---
p_max = (gimbal_slew / a) .^ (1/alpha);

% --- Build the table ---
T = table(servo_names, slew_rated, prices, gimbal_slew, p_max, ...
    'VariableNames', {'servo','slew_rated_dps','price_usd','gimbal_slew_loaded_dps','p_max_predicted'});
T = sortrows(T, 'price_usd');

fprintf('Predicted p_max stabilizable per servo (loaded, %.0fx linkage, %.0f%% derate):\n\n', linkage_ratio, derate*100);
fprintf('  %-25s %5s  %5s  %8s  %8s\n', 'Servo', '$', 'rated', 'gimbal', 'p_max');
fprintf('  %s\n', repmat('-',1,65));
for i = 1:height(T)
    fprintf('  %-25s %4d  %5d  %8.1f  %8.2f\n', T.servo{i}, T.price_usd(i), ...
        T.slew_rated_dps(i), T.gimbal_slew_loaded_dps(i), T.p_max_predicted(i));
end

% --- Pareto front (drop dominated points) ---
T2 = sortrows(T, 'price_usd');
keep = true(height(T2),1);
best_p = -inf;
for i = 1:height(T2)
    if T2.p_max_predicted(i) > best_p
        best_p = T2.p_max_predicted(i);
    else
        keep(i) = false;
    end
end
Tp = T2(keep,:);

fprintf('\nPARETO FRONTIER ($/p_max):\n');
fprintf('  %-25s %5s  %8s\n', 'Servo', '$', 'p_max');
fprintf('  %s\n', repmat('-',1,42));
for i = 1:height(Tp)
    fprintf('  %-25s %4d  %8.2f\n', Tp.servo{i}, Tp.price_usd(i), Tp.p_max_predicted(i));
end

writetable(T,  fullfile(here,'results','servo_pareto_all.csv'));
writetable(Tp, fullfile(here,'results','servo_pareto_frontier.csv'));
fprintf('\nSaved: experiments/results/servo_pareto_all.csv\n');
fprintf('Saved: experiments/results/servo_pareto_frontier.csv\n');
