package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class ClawOpen implements Action {
    private Claw claw;
    private Telemetry telemetry;
    public ClawOpen(Claw claw, Telemetry telemetry) {
        this.claw = claw;
        this.telemetry = telemetry;
    }

    /**
     * Opens the claw
     * @param telemetryPacket
     * @return
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        claw.intakeClawOpen();
        telemetry.addLine("Claw open");
        return false;
    }
}
