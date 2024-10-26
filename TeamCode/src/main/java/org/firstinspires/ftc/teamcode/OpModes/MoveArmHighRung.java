package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;

public class MoveArmHighRung implements Action {
    public OpMode opMode;

    Arm arm = new Arm(opMode);

    public MoveArmHighRung(OpMode opMode){
        this.opMode = opMode;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        arm.moveArmHighRungPosition();
        return true;
    }
}
