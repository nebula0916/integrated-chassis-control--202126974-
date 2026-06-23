function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
% CTRL_VERTICAL - 에너지 방향 기반 Skyhook CDC(가변감쇠) 수직 제어기
% 입력  : suspState.zs_dot[4x1] 차체 수직속도, suspState.zu_dot[4x1] 바퀴 수직속도,
%         ctrlState 상태저장, CTRL 게인, dt[s] 스텝
% 출력  : dampingCmd[4x1, Ns/m] 휠별 감쇠계수
% 기법  : Skyhook 조건부 가변감쇠 (cMin~cMax 캡)

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    skyGain = CTRL.VER.skyGain;

    dampingCmd = zeros(4, 1);

    for i = 1:4
        % 4륜 입력(센서값)
        zs_dot = suspState.zs_dot(i); % 차체 수직 속도
        zu_dot = suspState.zu_dot(i); % 타이어 수직 속도

        % SkyHook 조건 판단 변수
        rel_vel = zs_dot - zu_dot; %서스펜션 간 상대속도
        sky_condition = zs_dot * rel_vel; %부호판단

        % 에너지 소산 방향이 일치할 때 댐퍼 활성화
        if sky_condition > 0
            % 발산 방지
            if abs(rel_vel) > 1e-4
                c_eq = skyGain * (abs(zs_dot) / abs(rel_vel));
            else
                c_eq = cMax;
            end
            % 물리적 하드웨어 한계 설정
            dampingCmd(i) = max(cMin, min(cMax, c_eq));
        else
            % 댐핑 방향이 엇갈려 차체가 흔들리는 경우 댐퍼를 min(부드럽게) 설정 
            dampingCmd(i) = cMin;
        end
    end
end