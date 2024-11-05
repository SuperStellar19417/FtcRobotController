package org.firstinspires.ftc.teamcode.OpModes.Action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;


public class RaiseArmHighBucket implements Action {

    private Arm arm;



    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
       // new Arm = new Arm(arm.HardwareMap, arm.telemetry)
        return false;
    }
}
