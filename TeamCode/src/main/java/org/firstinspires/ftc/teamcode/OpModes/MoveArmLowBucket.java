package org.firstinspires.ftc.teamcode.OpModes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;

public class MoveArmLowBucket implements Action {
    public OpMode opMode;

    Arm arm = new Arm(opMode);

    public MoveArmLowBucket(OpMode opMode){
        this.opMode = opMode;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        try {
            arm.moveArmLowBucketPosition();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        return true;
    }
}
