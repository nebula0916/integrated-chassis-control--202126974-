function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
% CTRL_COORDINATOR - 횡/종/수직 명령을 액추에이터로 배분하는 통합 조율기
% 입력  : latCmd(steerAngle,yawMoment), lonCmd(Fx_total,abs_adj), verCmd[4x1],
%         vx[m/s], VEH/CTRL/LIM 파라미터
% 출력  : actuatorCmd.steerAngle[rad], .brakeTorque[4x1,Nm], .dampingCoeff[4x1]
% 기법  : 종제동 전후배분 + ESC 차동제동 allocation + RSC + CDC 롤 스태빌라이저
% 비고  : MIMO 채널 결합을 처리하는 유일 지점

    % 종방향 기본 제동 배분 (65 대 35)
    Fx = lonCmd.Fx_total;
    brakeTorque_base = zeros(4,1);
    if Fx < 0 % 감속 목표
        T_total = abs(Fx) * VEH.rw; % 힘->토크 변환
        T_f = (T_total * 0.65) / 2; % 전륜 65% 분배
        T_r = (T_total * 0.35) / 2; % 후련 65% 분배
        brakeTorque_base = [T_f; T_f; T_r; T_r];
    end

    % ESC 횡방향 차동 제동기 - Yaw moment를 좌우 제동력 차이로 구현
    Mz = latCmd.yawMoment;
    brakeTorque_esc = zeros(4,1);
    if abs(Mz) > 10 
        dT_f = (abs(Mz) * 0.65) / (VEH.track_f / 2) * VEH.rw;
        dT_r = (abs(Mz) * 0.35) / (VEH.track_r / 2) * VEH.rw;
        if Mz > 0 % Yaw moment 부호에 따른 제동 바퀴 선택
            brakeTorque_esc = [dT_f; 0; dT_r; 0]; 
        else
            brakeTorque_esc = [0; dT_f; 0; dT_r]; 
        end
    end
    
    % RSC 전복 방지- 극심한 요동에서 4륜 모두 강하게 제동
    brakeTorque_rsc = zeros(4,1);
    if abs(Mz) > 2000
        brakeTorque_rsc = [1000; 1000; 600; 600]; %LTR 억제
    end


    % longitudinal ABS 보정과 합산
    T_combined = brakeTorque_base + brakeTorque_esc + brakeTorque_rsc;
    
    T_final = T_combined;
    if isfield(lonCmd, 'abs_adj')
        T_final = T_final + lonCmd.abs_adj; 
    end

    % 하한선 -5000 -> 강제 제동에 대한 토크를 ABS가 상쇄 가능하도록 여유 확보
    final_brakeTorque = max(-5000, min(LIM.MAX_BRAKE_TRQ, T_final));

    % CDC Rolling Stablizer - 선회 시 4륜 댐퍼 최댓값 설정으로 Rolling 억제
    final_verCmd = verCmd;
    if abs(latCmd.steerAngle) > deg2rad(1.0) || abs(latCmd.yawMoment) > 500
        final_verCmd = ones(4,1) * CTRL.VER.cMax;
    end
    
    % 출력 Mapping (MIMO)
    actuatorCmd.steerAngle   = max(-LIM.MAX_STEER_ANGLE, min(LIM.MAX_STEER_ANGLE, latCmd.steerAngle));
    actuatorCmd.brakeTorque  = final_brakeTorque;
    actuatorCmd.dampingCoeff = final_verCmd; 
end