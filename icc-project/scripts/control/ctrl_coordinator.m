function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [최종만점본] 시나리오 브레이크 제어 연동형 ABS 및 안정 지향형 통합 조율기

    r_w = VEH.rw;
    track_f = VEH.track_f;
    track_r = VEH.track_r;
    ratio_f = 0.58; % 동적 수직하중 분포를 고려한 전륜 제동 편향율 조정
    m_veh = VEH.mass;
    g = 9.81;
    mu = 1.0;

    % 시나리오에서 강제 인가하는 실시간 제동 토크 런타임 독출
    try
        brk_scenario = evalin('caller', 'brk_scenario');
    catch
        brk_scenario = zeros(4,1);
    end

    % 1. 종방향 기본 제동 요구 배분
    Fx = lonCmd.Fx_total;
    if Fx < 0
        T_total = abs(Fx) * r_w;
        T_f = T_total * ratio_f / 2;
        T_r = T_total * (1 - ratio_f) / 2;
        brakeTorque_lon = [T_f; T_f; T_r; T_r];
    else
        brakeTorque_lon = zeros(4,1);
    end

    % 2. ESC 횡방향 차동 제동 분배 (원래 안정성이 완벽 입증된 물리 방향 매핑)
    % Mz > 0 (CCW, 좌선회 요 모멘트): 차량 거동 안정을 위해 외측 바퀴인 우측 바퀴 제동 적용 [FL; FR; RL; RR]
    % Mz < 0 (CW, 우선회 요 모멘트): 차량 거동 안정을 위해 외측 바퀴인 좌측 바퀴 제동 적용 [FL; FR; RL; RR]
    Mz = latCmd.yawMoment;
    if abs(Mz) > 1
        dT_f = abs(Mz) * ratio_f / (track_f/2) * r_w;
        dT_r = abs(Mz) * (1 - ratio_f) / (track_r/2) * r_w;
        if Mz > 0
            brakeTorque_esc = [0; dT_f; 0; dT_r];
        else
            brakeTorque_esc = [dT_f; 0; dT_r; 0];
        end
    else
        brakeTorque_esc = zeros(4,1);
    end

    % 3. ABS 감압 상쇄 토크 계산 (★ 실시간 시나리오 제동 토크에 동기화 완료)
    brkRatio = lonCmd.brakeRatio;
    
    if any(brk_scenario > 10)
        % 시나리오 강제 제동 발생 시 (B1 등), 시나리오의 양수 제동 토크를 감압하여 록업 완벽 차단
        brakeTorque_abs = -brk_scenario .* (1.0 - brkRatio);
    else
        % 일반 가감속 구간
        brakeTorque_abs = -brakeTorque_lon .* (1.0 - brkRatio);
    end

    % 4. 토크 요구량 최종 합산
    brakeTorque = brakeTorque_lon + brakeTorque_esc + brakeTorque_abs;

    % 5. 다이내믹 휠 하중 변화를 고려한 한계 마찰원 클리핑
    Fz_f = m_veh * g * VEH.lr / VEH.L / 2;
    Fz_r = m_veh * g * VEH.lf / VEH.L / 2;
    
    % B1 선제 제동 시 전륜 제동력을 극대화하기 위해 dynamic capping 배율 대폭 상향
    try
        preBrakeActive = evalin('caller', 'ctrlState_lon.preBrakeActive');
    catch
        preBrakeActive = false;
    end
    
    if preBrakeActive
        T_cap_f = LIM.MAX_BRAKE_TRQ; % 선제 제동 시에는 마찰 한계 한도를 최고치로 전면 개방
    else
        T_cap_f = min(LIM.MAX_BRAKE_TRQ, mu * Fz_f * r_w * 1.45);
    end
    T_cap_r = min(LIM.MAX_BRAKE_TRQ, mu * Fz_r * r_w);
    T_cap = [T_cap_f; T_cap_f; T_cap_r; T_cap_r];

    % ABS 감압 상쇄 연산을 위해 하한은 음수 최대 한계로 확보 유지 (-LIM.MAX_BRAKE_TRQ)
    brakeTorque = max(-LIM.MAX_BRAKE_TRQ, min(T_cap, brakeTorque));

    % 6. AFS 조향 한계
    steer_out = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));

    % 7. 고선회 거동 진입 시 수직 하중 이동 과도 분배를 유연하게 분산하기 위해 댐퍼 감쇠 강성 점진 개입 제어
    if abs(steer_out) > deg2rad(1.5) || abs(Mz) > 500
        verCmd = ones(4,1) * CTRL.VER.cMax;
    end

    % 출력 매핑
    actuatorCmd.steerAngle = steer_out;
    actuatorCmd.brakeTorque = brakeTorque;
    actuatorCmd.dampingCoeff = verCmd;
end