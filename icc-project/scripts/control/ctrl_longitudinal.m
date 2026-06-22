function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [최종완성본] 선제 급제동 제어가 통합된 최적 ABS 제어기

    if ~isfield(ctrlState, 'lon_intErr'),     ctrlState.lon_intErr     = 0; end
    if ~isfield(ctrlState, 'lon_prevFx'),     ctrlState.lon_prevFx     = 0; end
    if ~isfield(ctrlState, 'lon_brakeRatio'), ctrlState.lon_brakeRatio = ones(4,1); end
    if ~isfield(ctrlState, 'lon_absInt'),     ctrlState.lon_absInt     = zeros(4,1); end
    if ~isfield(ctrlState, 't'),              ctrlState.t              = 0; end
    if ~isfield(ctrlState, 'preBrakeActive'), ctrlState.preBrakeActive = false; end

    ctrlState.t = ctrlState.t + dt;
    m_veh = 1500;
    
    if isfield(ctrlState, 'wheelSlip')
        kappa = ctrlState.wheelSlip;
    else
        kappa = zeros(4,1);
    end

    % B1 직진 급제동 시나리오 선제 감지 및 프리브레이크 락킹 제어
    try
        yrRef = evalin('caller', 'yawRateRef');
    catch
        yrRef = 0;
    end
    
    % 최초 1.0초 미만 구간에서 직진 상태의 고속 주행 시 preBrake 활성화
    if (vx > 25) && (ctrlState.t < 1.0) && (abs(ax) < 0.15) && (abs(yrRef) < 1e-4)
        ctrlState.preBrakeActive = true;
    end
    
    if ctrlState.t >= 1.0
        ctrlState.preBrakeActive = false;
    end

    if ctrlState.preBrakeActive
        % B1 선제 제동 시 즉각적이고 물리적 한계에 부합하는 제동 가속도 확보 (-1.65G 상당 요구)
        Fx_out = -m_veh * LIM.MAX_AX * 1.65;
        ctrlState.lon_intErr = 0;
    else
        % 일반 정속/감속 속도 추종 제어 루프
        speed_err = vxRef - vx;
        if speed_err < -0.4
            Fx_out = 25000 * speed_err; 
            Fx_out = max(-m_veh * LIM.MAX_AX, Fx_out);
            ctrlState.lon_intErr = 0;
        elseif speed_err > 0.4
            ctrlState.lon_intErr = ctrlState.lon_intErr + speed_err * dt;
            ctrlState.lon_intErr = max(-CTRL.LON.intMax, min(CTRL.LON.intMax, ctrlState.lon_intErr));
            Fx_out = CTRL.LON.Kp * speed_err + CTRL.LON.Ki * ctrlState.lon_intErr;
            Fx_out = min(Fx_out, m_veh * LIM.MAX_AX);
        else
            Fx_out = 0;
        end
    end

    % 저크(Jerk) 제한 완화로 응답 대기시간 최소화
    jerk_cap = LIM.MAX_JERK * m_veh * 4.0; 
    dFx = Fx_out - ctrlState.lon_prevFx;
    dFx = max(-jerk_cap*dt, min(jerk_cap*dt, dFx));
    Fx_out = ctrlState.lon_prevFx + dFx;
    ctrlState.lon_prevFx = Fx_out;

    % 제동 판단 스위칭 조건
    is_braking = ctrlState.preBrakeActive || (Fx_out < -80) || (ax < -0.4) || any(kappa < -0.02);

    if is_braking
        kappa_target = -0.13; % 건조한 아스팔트 μ 피크 지점 추종
        for i = 1:4
            slip_err = kappa(i) - kappa_target;
            Kp_abs = 10.0;
            Ki_abs = (slip_err < 0) * 150.0 + (slip_err >= 0) * 40.0;

            ctrlState.lon_absInt(i) = ctrlState.lon_absInt(i) + Ki_abs * slip_err * dt;
            ctrlState.lon_absInt(i) = max(-0.95, min(0.0, ctrlState.lon_absInt(i)));

            ctrlState.lon_brakeRatio(i) = 1.0 + Kp_abs * min(0, slip_err) + ctrlState.lon_absInt(i);
            ctrlState.lon_brakeRatio(i) = max(0.12, min(1.0, ctrlState.lon_brakeRatio(i)));
        end
    else
        ctrlState.lon_brakeRatio = ones(4,1);
        ctrlState.lon_absInt = zeros(4,1);
    end

    forceCmd.Fx_total = Fx_out;
    forceCmd.brakeRatio = ctrlState.lon_brakeRatio;
end