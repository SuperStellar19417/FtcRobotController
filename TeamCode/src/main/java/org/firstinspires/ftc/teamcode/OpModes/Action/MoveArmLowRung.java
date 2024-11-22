package org.firstinspires.ftc.teamcode.OpModes.Action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;

public class MoveArmLowRung implements Action {
    private final OpMode opMode;
    private final Arm arm;

    public MoveArmLowRung(OpMode opMode){
        this.opMode = opMode;
        arm = new Arm(opMode);
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        arm.moveArmLowRungPosition();
        return true;
    }
}
