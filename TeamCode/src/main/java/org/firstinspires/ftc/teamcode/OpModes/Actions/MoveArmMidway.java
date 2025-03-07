package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;

public class MoveArmMidway implements Action {

    private Arm arm;
    private boolean initialized = false;
    private Telemetry telemetry;

    public MoveArmMidway(Arm arm, Telemetry telemetry){
        this.arm = arm;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        // powers on motor, if it is not on
        if (!initialized) {
            arm.moveArmMidwayPosition();
            initialized = true;
        }

        // checks arm and slide current position
        double pos = arm.armMotor.getCurrentPosition();
        telemetry.addData("Arm pos:", pos);
        telemetry.update();

        if (pos < 600 - 50) {
            // true causes the action to rerun
            return true;
        } else {
            // false stops action rerun
            telemetry.addLine("ARM reached midway position");
            telemetry.update();
            return false;
        }

    }
}
