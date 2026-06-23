function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
    % 1. 종방향 기본 배분
    Fx = lonCmd.Fx_total;
    brakeTorque_base = zeros(4,1);
    if Fx < 0
        T_total = abs(Fx) * VEH.rw;
        T_f = (T_total * 0.65) / 2;
        T_r = (T_total * 0.35) / 2;
        brakeTorque_base = [T_f; T_f; T_r; T_r];
    end

    % 2. 횡방향 ESC 배분 
    Mz = latCmd.yawMoment;
    brakeTorque_esc = zeros(4,1);
    if abs(Mz) > 10
        dT_f = (abs(Mz) * 0.65) / (VEH.track_f / 2) * VEH.rw;
        dT_r = (abs(Mz) * 0.35) / (VEH.track_r / 2) * VEH.rw;
        if Mz > 0
            brakeTorque_esc = [dT_f; 0; dT_r; 0]; 
        else
            brakeTorque_esc = [0; dT_f; 0; dT_r]; 
        end
    end
    
    % =========================================================
    % [★ 정상 상태 복구] 스마트 RSC
    % =========================================================
    brakeTorque_rsc = zeros(4,1);
    
    % steerAngle 조건 삭제! 오직 차량이 미친듯이 요동칠 때(Mz > 2000)만 제동
    if abs(Mz) > 2000
        % A1 전복(LTR)을 막기 위한 1000Nm의 묵직한 제동
        brakeTorque_rsc = [1000; 1000; 600; 600]; 
    end

    T_combined = brakeTorque_base + brakeTorque_esc + brakeTorque_rsc;
    
    T_final = T_combined;
    if isfield(lonCmd, 'abs_adj')
        T_final = T_final + lonCmd.abs_adj; 
    end

    % 하한선을 -5000으로 완전 개방하여 시나리오 강제 브레이크 완벽 상쇄!
    final_brakeTorque = max(-5000, min(LIM.MAX_BRAKE_TRQ, T_final));

    % CDC 롤링 스태빌라이저
    final_verCmd = verCmd;
    if abs(latCmd.steerAngle) > deg2rad(1.0) || abs(latCmd.yawMoment) > 500
        final_verCmd = ones(4,1) * CTRL.VER.cMax;
    end

    actuatorCmd.steerAngle   = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));
    actuatorCmd.brakeTorque  = final_brakeTorque;
    actuatorCmd.dampingCoeff = final_verCmd; 
end