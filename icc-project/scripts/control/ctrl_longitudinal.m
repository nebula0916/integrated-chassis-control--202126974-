function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
% CTRL_LONGITUDINAL - 속도추종 PI(+가속도 FF) + 역토크 ABS 종방향 제어기
% 입력  : vxRef[m/s] 목표속도, vx[m/s] 실제속도, ax[m/s^2] 종가속도,
%         ctrlState 상태저장, CTRL/LIM 게인·한계, dt[s] 스텝
% 출력  : forceCmd.Fx_total[N] 종방향 힘, forceCmd.abs_adj[4x1,Nm] ABS 보정토크
% 기법  : 속도 PI + accel feedforward + jerk rate-limit + slip-ratio ABS

    % 미적값 초기화(누적오류 방지)
    if ~isfield(ctrlState, 'lon_intErr'), ctrlState.lon_intErr = 0; end
    if ~isfield(ctrlState, 'prev_Fx'),    ctrlState.prev_Fx    = 0; end
    
    % [제어기 1] - Speed Control PI + 속도 FeedForward
    speed_err = vxRef - vx; % Speed error e(t)
    Fx_req = CTRL.LON.Kp * speed_err + CTRL.LON.Ki * ctrlState.lon_intErr - 500 * ax;
    
    % 힘 변화량을 제한하는 rate limiter
    max_dFx = LIM.MAX_JERK * 1500 * dt; % 1 step 당 최대 힘 변화량
    Fx_out = ctrlState.prev_Fx + max(-max_dFx, min(max_dFx, Fx_req - ctrlState.prev_Fx));
    Fx_out = max(-1500 * LIM.MAX_AX, min(1500 * LIM.MAX_AX, Fx_out)); % 최대 힘값으로 제한
    ctrlState.prev_Fx = Fx_out;

    % 휠 Slip Rate kappa 도출 
    if isfield(ctrlState, 'wheelSlip')
        kappa = ctrlState.wheelSlip;
    else
        kappa = zeros(4,1);
    end
    
    % [제어기 2] - 역 토크 ABS - Slip Rate가 목표값보다 깊다면 제동으로 감압
    abs_adj = zeros(4,1);
    kappa_target = -0.14;

    for i = 1:4
        if kappa(i) < kappa_target  % 목표보다 깊으면 역토크
            abs_adj(i) = 50000 * (kappa(i) - kappa_target); 
        end
    end

    forceCmd.Fx_total = Fx_out;
    forceCmd.abs_adj  = abs_adj;
end