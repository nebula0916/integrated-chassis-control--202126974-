function [dampingCmd, ctrlState] = ctrl_vertical(suspState, ctrlState, CTRL, dt)
%CTRL_VERTICAL [최종본] CDC 가변 감쇠 제어

    cMin = CTRL.VER.cMin;
    cMax = CTRL.VER.cMax;
    skyGain = CTRL.VER.skyGain;

    dampingCmd = zeros(4, 1);

    for i = 1:4
        zs_dot = suspState.zs_dot(i);
        zu_dot = suspState.zu_dot(i);

        rel_vel = zs_dot - zu_dot;
        sky_condition = zs_dot * rel_vel;

        if sky_condition > 0
            if abs(rel_vel) > 1e-4
                c_eq = skyGain * (abs(zs_dot) / abs(rel_vel));
            else
                c_eq = cMax;
            end
            dampingCmd(i) = max(cMin, min(cMax, c_eq));
        else
            dampingCmd(i) = cMin;
        end
    end
end
