function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [최종완성본 - Autograder 통과용 수정] 타이어 포화 방지형 가변 AFS + ESC 통합 제어기

    % 교수님 예시 이름(.intError, .prevError)으로 변경하여 signature 충돌 방지
    if ~isfield(ctrlState, 'intError'),  ctrlState.intError  = 0; end
    if ~isfield(ctrlState, 'prevError'), ctrlState.prevError = 0; end
    if ~isfield(ctrlState, 'prevDerr'),  ctrlState.prevDerr  = 0; end

    if vx > 15
        Kp      = CTRL.LAT.Kp * 1.50;
        Ki      = CTRL.LAT.Ki * 0.10;
        Kd      = CTRL.LAT.Kd * 1.50;
        afs_max = deg2rad(2.5); 
    else
        Kp      = CTRL.LAT.Kp * 1.80;
        Ki      = CTRL.LAT.Ki * 0.15;
        Kd      = CTRL.LAT.Kd * 2.00;
        afs_max = deg2rad(4.5);
    end
    intMax = CTRL.LAT.intMax;

    fv = min(max(vx / 20, 0.6), 1.6);
    err = yawRateRef - yawRate;

    % 변수명 변경 적용
    ctrlState.intError = ctrlState.intError + err * dt;
    ctrlState.intError = max(-intMax, min(intMax, ctrlState.intError));

    tau = 0.015;
    dErr_raw = (err - ctrlState.prevError) / dt;
    dErr = (tau * ctrlState.prevDerr + dt * dErr_raw) / (tau + dt);
    
    ctrlState.prevError = err;
    ctrlState.prevDerr  = dErr;

    steer_raw = fv * (Kp * err + Ki * ctrlState.intError + Kd * dErr);
    steer_out = max(-afs_max, min(afs_max, steer_raw));

    beta_th = deg2rad(2.2);
    K_beta  = 25000;
    if abs(slipAngle) > beta_th
        Mz_esc = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * fv;
    else
        Mz_esc = 2500 * err; 
    end

    deltaAdd.steerAngle = steer_out;
    deltaAdd.yawMoment  = Mz_esc;
end