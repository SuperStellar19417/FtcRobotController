package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class WristToIntakePosition implements Action {
    private Claw claw;
    private Telemetry telemetry;

    public WristToIntakePosition(Claw claw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.claw = claw;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        claw.wristMid();
        telemetry.addLine("Wrist moved to mid pos");
        return false;
    }
}