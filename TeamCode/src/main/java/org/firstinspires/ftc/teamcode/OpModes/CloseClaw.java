package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class CloseClaw implements Action {
    Telemetry telemetry;
    Claw claw = new Claw(hardwareMap, telemetry);

    @Override

    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        claw.intakeClawClose();
        return true;
    }
}
