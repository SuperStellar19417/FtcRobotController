package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;

public class MoveArmHighRung implements Action {

    private final Arm arm;
    private final OpMode opMode;

    public MoveArmHighRung(OpMode opMode){

        this.opMode = opMode;
        arm = new Arm(opMode);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        arm.moveArmHighRungPosition();
        return true;
    }
}
