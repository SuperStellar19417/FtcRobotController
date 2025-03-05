package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;

public class SlidesRetractToMin implements Action {

    private IntakeSlide intakeSlide;
    private boolean initialized = false;
    private Telemetry telemetry;

    public SlidesRetractToMin(IntakeSlide intakeSlide, Telemetry telemetry){
        this.intakeSlide = intakeSlide;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        if (!initialized) {
            intakeSlide.moveSlideLow();
            initialized = true;
        }

        // checks arm and slide current position
        double pos = intakeSlide.slideMotor.getCurrentPosition();
        telemetry.addData("Slide pos:", pos);
        telemetry.update();

        if (pos > intakeSlide.SLIDE_POSITION_MIN + 200) {
            telemetry.addData("Slide pos:", pos);
            telemetry.update();
            // true causes the action to rerun
            return true;
        } else {
            telemetry.addLine("Slides are down");
            telemetry.update();
            // false stops action rerun
            return false;
        }
    }
}