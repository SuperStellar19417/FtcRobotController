package org.firstinspires.ftc.teamcode.OpModes.Action;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;

public class OpenClaw implements Action {
    public OpMode opMode;

    Claw joeBiden = new Claw(opMode);

    public OpenClaw(OpMode opMode) {
        this.opMode = opMode;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        joeBiden.intakeClawOpen();
        return true;

    }

}
