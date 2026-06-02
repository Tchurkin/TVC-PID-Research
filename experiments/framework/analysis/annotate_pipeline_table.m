function T = annotate_pipeline_table(T, P, layer_name, dataset_name)
%ANNOTATE_PIPELINE_TABLE Add canonical pipeline metadata and fixed parameters.

if nargin < 4
    dataset_name = "";
end
if nargin < 3
    layer_name = "";
end

contract = pipeline_contract();
meta = struct();
meta.pipeline_layer = string(layer_name);
meta.dataset_name = string(dataset_name);
meta.schema_version = string(contract.version);
meta.mass = P.rocket.mass;
meta.Iyy = P.rocket.Iyy;
meta.static_margin = P.rocket.static_margin;
meta.Cm_alpha = P.rocket.Cm_alpha;
meta.control_effectiveness = P.rocket.control_effectiveness;
meta.thrust = P.rocket.thrust;
meta.deadband = P.rocket.deadband;
meta.backlash = P.rocket.backlash;
meta.latency = P.rocket.latency;
meta.wind_strength = P.rocket.wind_strength;
meta.nominal_servo_slew_deg_s = P.rocket.servo_slew;
meta.nominal_max_gimbal_deg = P.rocket.max_gimbal;

meta_names = fieldnames(meta);
for i = 1:numel(meta_names)
    name = meta_names{i};
    value = meta.(name);
    if ~ismember(name, T.Properties.VariableNames)
        T.(name) = repmat(value, height(T), 1);
    end
end

if ~ismember('parameter_vector_id', T.Properties.VariableNames)
    T.parameter_vector_id = (1:height(T))';
end
end
