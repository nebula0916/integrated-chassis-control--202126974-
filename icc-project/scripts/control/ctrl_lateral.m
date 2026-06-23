function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
% CTRL_LATERAL - AFS(yaw-rate 추종 PID) + ESC(slip 기반 DYC) 횡방향 제어기
% 입력  : yawRateRef[rad/s] 목표요레이트, yawRate[rad/s] 실제요레이트,
%         slipAngle[rad] 차체슬립각 β, vx[m/s] 종속도, ctrlState 상태저장,
%         CTRL/LIM 게인·한계, dt[s] 스텝
% 출력  : deltaAdd.steerAngle[rad] 보조조향, deltaAdd.yawMoment[Nm] ESC 요모멘트
% 기법  : PID + LPV gain scheduling(fv) + slip deadzone DYC
% 비고  : ESC 부호는 coordinator brake 매핑과 정합 필요 (분리 시 LTR 악화)
    
    % 미적값 초기화(누적오류 방지)
    if ~isfield(ctrlState, 'prevError'), ctrlState.prevError = 0; end 
    if ~isfield(ctrlState, 'prevDerr'),  ctrlState.prevDerr  = 0; end
    if ~isfield(ctrlState, 'intError'),  ctrlState.intError  = 0; end
    
    % LPV Gain Scheduling - 저속에서 보강, 고속에서 감쇠하는 변수값
    fv = min(max(vx / 20.0, 0.5), 1.5);
    
    err = yawRateRef - yawRate; %추종 오차 e(t)
    tau = 0.02;
    dErr_raw = (err - ctrlState.prevError) / dt; % 미분기
    dErr = (tau * ctrlState.prevDerr + dt * dErr_raw) / (tau + dt); % 1st Order LPF 미분기
    
    % 다음 미분을 위한 변화량 저장
    ctrlState.prevError = err; 
    ctrlState.prevDerr  = dErr;

    % [제어기 1] AFS — yaw-rate tracking PID
    int_weight = max(0, 1.0 - abs(slipAngle) / deg2rad(1.5)); % Silp Angle(Beta)값이 커질수록 적분 가중치 감소
    ctrlState.intError = ctrlState.intError + (int_weight * err) * dt; 
    ctrlState.intError = max(-CTRL.LAT.intMax, min(CTRL.LAT.intMax, ctrlState.intError));
    
    %AFS Output - fv로 나눠 고속에서 Control 게인 감쇠 및 물리적 한계에 포화
    steer_raw = (CTRL.LAT.Kp * err + CTRL.LAT.Ki * ctrlState.intError + CTRL.LAT.Kd * dErr) / fv;
    steer_out = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, steer_raw));

    % [제어기 2] ESC (Beta값 기반 Yaw Rate Controller)
    beta_th = deg2rad(1.2);
    K_beta  = 45000;
    Mz_slip = K_beta * sign(slipAngle) * max(0, abs(slipAngle) - beta_th) * fv;

    % 데드존 2.0도 — Yaw Error가 2.0도 보다 커질 때 동작
    if abs(err) > deg2rad(2.0)
        Mz_yaw = 5000 * err * fv;
        Mz_yaw = max(-4000, min(4000, Mz_yaw));
    else
        Mz_yaw = 0;
    end
    
    %두 Yaw Moment 합산
    deltaAdd.steerAngle = steer_out;
    deltaAdd.yawMoment  = Mz_slip + Mz_yaw;
end
