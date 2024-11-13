package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class DownWrist implements Action {
    public OpMode opMode;

    public DownWrist(OpMode opMode) {
        this.opMode = opMode;
    }

    Claw claw = new Claw(opMode);

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        return true;
    }
}