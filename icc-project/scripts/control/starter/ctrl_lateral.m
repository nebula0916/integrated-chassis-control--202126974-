function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [최종완성본] 타이어 포화 방지형 가변 AFS + ESC 통합 제어기

    if ~isfield(ctrlState, 'lat_intErr'),   ctrlState.lat_intErr   = 0; end
    if ~isfield(ctrlState, 'lat_prevErr'),  ctrlState.lat_prevErr  = 0; end
    if ~isfield(ctrlState, 'lat_prevDerr'), ctrlState.lat_prevDerr = 0; end

    % 대역폭 최적화 및 타이어 횡력 포화 방지를 위한 최적의 조향 제어 게인 설계
    if vx > 15
        Kp      = CTRL.LAT.Kp * 1.50;
        Ki      = CTRL.LAT.Ki * 0.10;
        Kd      = CTRL.LAT.Kd * 1.50;
        afs_max = deg2rad(2.5); % 고속 영역 횡력 보존을 위한 최적의 조향 마진
    else
        Kp      = CTRL.LAT.Kp * 1.80;
        Ki      = CTRL.LAT.Ki * 0.15;
        Kd      = CTRL.LAT.Kd * 2.00;
        afs_max = deg2rad(4.5);
    end
    intMax = CTRL.LAT.intMax;

    fv = min(max(vx / 20, 0.6), 1.6);
    err = yawRateRef - yawRate;

    % 적분 안티와인드업 및 필터형 미분기
    ctrlState.lat_intErr = ctrlState.lat_intErr + err * dt;
    ctrlState.lat_intErr = max(-intMax, min(intMax, ctrlState.lat_intErr));

    tau = 0.015;
    dErr_raw = (err - ctrlState.lat_prevErr) / dt;
    dErr = (tau * ctrlState.lat_prevDerr + dt * dErr_raw) / (tau + dt);
    ctrlState.lat_prevErr  = err;
    ctrlState.lat_prevDerr = dErr;

    steer_raw = fv * (Kp * err + Ki * ctrlState.lat_intErr + Kd * dErr);
    steer_out = max(-afs_max, min(afs_max, steer_raw));

    % ESC 복원 모멘트 설계 (원래 입증된 안정형 부호 컨벤션 복원 및 평상시 선회 저항 80% 감축)
    beta_th = deg2rad(2.2);
    K_beta  = 25000;
    if abs(slipAngle) > beta_th
        % 슬립 한계 초과 시 복원 요 모멘트 적용 (원래 검증된 안정형 부호 복구)
        Mz_esc = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * fv;
    else
        % 일반 안정 상태에서는 ESC 제동 개입을 최소화하여 횡방향 그립 최대 보존
        Mz_esc = 2500 * err; 
    end

    deltaAdd.steerAngle = steer_out;
    deltaAdd.yawMoment  = Mz_esc;
end