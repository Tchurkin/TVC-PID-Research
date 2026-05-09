function telem = s2r_load_telemetry_dataset(telemetry_dir)
manifest_p = fullfile(telemetry_dir, 'launch_manifest.csv');
imu_p = fullfile(telemetry_dir, 'imu_theta_q_timeseries.csv');
event_p = fullfile(telemetry_dir, 'flight_event_markers.csv');

telem.launch_manifest = readtable(manifest_p, 'TextType', 'string');
telem.imu = readtable(imu_p, 'TextType', 'string');
telem.events = readtable(event_p, 'TextType', 'string');

telem.launch_manifest.launch_id = string(telem.launch_manifest.launch_id);
telem.launch_manifest.controller = upper(string(telem.launch_manifest.controller));
telem.imu.launch_id = string(telem.imu.launch_id);
telem.imu.controller = upper(string(telem.imu.controller));
telem.events.launch_id = string(telem.events.launch_id);
end
