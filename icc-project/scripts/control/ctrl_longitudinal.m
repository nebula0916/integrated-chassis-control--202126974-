function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
    if ~isfield(ctrlState, 'lon_intErr'), ctrlState.lon_intErr = 0; end
    if ~isfield(ctrlState, 'prev_Fx'),    ctrlState.prev_Fx    = 0; end
    
    speed_err = vxRef - vx;
    Fx_req = CTRL.LON.Kp * speed_err + CTRL.LON.Ki * ctrlState.lon_intErr - 500 * ax;
    
    max_dFx = LIM.MAX_JERK * 1500 * dt;
    Fx_out = ctrlState.prev_Fx + max(-max_dFx, min(max_dFx, Fx_req - ctrlState.prev_Fx));
    Fx_out = max(-1500 * LIM.MAX_AX, min(1500 * LIM.MAX_AX, Fx_out));
    ctrlState.prev_Fx = Fx_out;

    % =========================================================
    % [★복구] 모듈 3: 역토크(Counter-Torque) ABS 
    % =========================================================
    if isfield(ctrlState, 'wheelSlip')
        kappa = ctrlState.wheelSlip;
    else
        kappa = zeros(4,1);
    end

    abs_adj = zeros(4,1);
    % [모듈 3] 역토크 ABS 
    kappa_target = -0.14; % -0.15는 너무 깊어 슬립이 터졌으므로 -0.13으로 조정

    for i = 1:4
        if kappa(i) < kappa_target
            abs_adj(i) = 50000 * (kappa(i) - kappa_target); 
        end
    end

    forceCmd.Fx_total = Fx_out;
    forceCmd.abs_adj  = abs_adj;
end