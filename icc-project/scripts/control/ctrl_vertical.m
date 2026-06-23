function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [최종본] 직관적 에너지 방향 기반 Skyhook CDC 제어기

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    skyGain = CTRL.VER.skyGain;

    dampingCmd = zeros(4, 1);

    for i = 1:4
        % 1. 센서 상태량 추출
        zs_dot = suspState.zs_dot(i); % 차체 수직 속도
        zu_dot = suspState.zu_dot(i); % 타이어 수직 속도

        % 2. 서스펜션 상대 속도 및 스카이훅 작동 조건 판단
        rel_vel = zs_dot - zu_dot;
        sky_condition = zs_dot * rel_vel;

        % 3. 에너지 소산 방향이 일치할 때만(양수) 가상 댐퍼 활성화
        if sky_condition > 0
            % 분모가 0이 되는 발산(Singularity) 방지
            if abs(rel_vel) > 1e-4
                c_eq = skyGain * (abs(zs_dot) / abs(rel_vel));
            else
                c_eq = cMax;
            end
            % 물리적 하드웨어 한계 캡핑
            dampingCmd(i) = max(cMin, min(cMax, c_eq));
        else
            % 방향이 엇갈려 차체를 오히려 흔들게 될 경우 댐퍼를 가장 부드럽게 개방
            dampingCmd(i) = cMin;
        end
    end
end