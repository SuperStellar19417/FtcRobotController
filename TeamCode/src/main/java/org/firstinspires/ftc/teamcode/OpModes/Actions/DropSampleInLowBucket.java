package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;

/**
 * Performs the action of dropping sample in low bucket.
 * Steps:
 *  1. Move arm to low bucket position
 *  2. Move slides to low bucket position
 *  3. Open Claw
 *  4. Wait for 1 second
 */
public class DropSampleInLowBucket implements Action {
    IntakeSlide intakeSlide;
    Arm arm;
    Claw claw;

    public DropSampleInLowBucket(IntakeSlide intakeSlide, Arm arm, Claw claw) {
        this.intakeSlide = intakeSlide;
        this.arm = arm;
        this.claw = claw;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        SleepAction oneSecondWait = new SleepAction(1);

        arm.moveArmLowBasketPosition();
        intakeSlide.moveSlideHigh();
        oneSecondWait.run(telemetryPacket);
        claw.wristMid();
        claw.intakeClawOpen();
        oneSecondWait.run(telemetryPacket);
        intakeSlide.moveSlideLow();

        return false;
    }

    public Action dropSampleInLowBucket(IntakeSlide intakeSlide, Arm arm, Claw claw) {
        return new DropSampleInLowBucket( intakeSlide, arm, claw);
    }
}


