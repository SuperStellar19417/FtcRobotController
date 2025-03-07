package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;

public class SlidesSlightlyExtend implements Action {
    private IntakeSlide intakeSlide;
    private boolean initialized = false;
    private Telemetry telemetry;

    private int slideMax = 400;

    public SlidesSlightlyExtend(IntakeSlide intakeSlide, Telemetry telemetry){
        this.intakeSlide = intakeSlide;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // powers on motor, if it is not on
        if (!initialized) {
            //  intakeSlide.moveSlideHigh();
            intakeSlide.extendSlide();
            initialized = true;
        }

        // checks arm and slide current position
        double pos = intakeSlide.slideMotor.getCurrentPosition();

        telemetry.addData("Slide pos:", pos);
        telemetry.update();

        // giving some buffer here for ticks
        if (pos < slideMax - 100) {
            // true causes the action to rerun
            return true;
        } else {
            telemetry.addLine("Slides extended");
            telemetry.update();

            // false stops action rerun
            return false;
        }
        // overall, the action powers the slides until it surpasses
        // SLIDE_POSITION_MAX encoder ticks, then powers it off

    }
}
