function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL AFS(yaw-rate PID) + ESC(sideslip/yaw DYC)  [안전판 — 59.51점 baseline]
%   입력: yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt
%   출력: deltaAdd.steerAngle [rad], deltaAdd.yawMoment [Nm]

    if ~isfield(ctrlState, 'prevError'), ctrlState.prevError = 0; end
    if ~isfield(ctrlState, 'prevDerr'),  ctrlState.prevDerr  = 0; end
    if ~isfield(ctrlState, 'intError'),  ctrlState.intError  = 0; end

    fv = min(max(vx / 20.0, 0.5), 1.5);

    err = yawRateRef - yawRate;
    tau = 0.02;
    dErr_raw = (err - ctrlState.prevError) / dt;
    dErr = (tau * ctrlState.prevDerr + dt * dErr_raw) / (tau + dt);

    ctrlState.prevError = err;
    ctrlState.prevDerr  = dErr;

    % AFS — yaw-rate tracking PID (slip 클 때 적분 가중 축소)
    int_weight = max(0, 1.0 - abs(slipAngle) / deg2rad(1.5));
    ctrlState.intError = ctrlState.intError + (int_weight * err) * dt;
    ctrlState.intError = max(-CTRL.LAT.intMax, min(CTRL.LAT.intMax, ctrlState.intError));

    steer_raw = (CTRL.LAT.Kp * err + CTRL.LAT.Ki * ctrlState.intError + CTRL.LAT.Kd * dErr) / fv;
    steer_out = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer_raw));

    % =========================================================
    % [제어기 2] ESC (A4 철통 방어 세팅)
    % =========================================================
    beta_th = deg2rad(1.2);
    K_beta  = 45000;
    Mz_slip = K_beta * sign(slipAngle) * max(0, abs(slipAngle) - beta_th) * fv;

    % 데드존 2.0도 — A4 정상 선회 시 DYC 비활성
    if abs(err) > deg2rad(2.0)
        Mz_yaw = 5000 * err * fv;
        Mz_yaw = max(-4000, min(4000, Mz_yaw));
    else
        Mz_yaw = 0;
    end

    deltaAdd.steerAngle = steer_out;
    deltaAdd.yawMoment  = Mz_slip + Mz_yaw;
end
