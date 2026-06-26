function cfg = configure_pid_controller(cfg, Kp, Kd)
%CONFIGURE_PID_CONTROLLER Set PID gains while preserving model limits.

cfg.controllers.PID.Kp = Kp;
cfg.controllers.PID.Ki = 0.0;
cfg.controllers.PID.Kd = Kd;
cfg.controllers.PID.u_max = cfg.plant.u_max;
cfg.controllers.PID.i_lim = cfg.plant.u_max;
