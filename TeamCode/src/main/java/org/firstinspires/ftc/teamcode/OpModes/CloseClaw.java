package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class CloseClaw implements Action {
    public OpMode opMode;

    Claw claw = new Claw(opMode);

    public CloseClaw(OpMode opMode) {
        this.opMode = opMode;
    }


    @Override

    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        claw.intakeClawClose();
        return true;
    }
}
