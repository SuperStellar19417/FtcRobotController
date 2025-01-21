package org.firstinspires.ftc.teamcode.OpModes.Actions;



import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class ClawClose implements Action {
    private Claw claw;
    private Telemetry telemetry;

    public ClawClose(Claw claw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.claw = claw;
    }

    /**
     * Closes the claw
     * @param telemetryPacket
     * @return
     */
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        claw.intakeClawClose();
        telemetry.addLine("Claw Close");
        return false;

    }

}
