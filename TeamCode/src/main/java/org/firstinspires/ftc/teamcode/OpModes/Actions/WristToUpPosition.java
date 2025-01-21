package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class WristToUpPosition implements Action {
    private Claw claw;
    private Telemetry telemetry;

    public WristToUpPosition(Claw claw, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.claw = claw;
    }

     @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        claw.wristUp();
        telemetry.addLine("Wrist moved to up pos");
        return true;
    }
}
