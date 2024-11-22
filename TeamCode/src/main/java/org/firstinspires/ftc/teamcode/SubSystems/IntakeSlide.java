package org.firstinspires.ftc.teamcode.SubSystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class IntakeSlide {

    private final double POWER_LEVEL_RUN = 0.8;
    private final double POWER_LEVEL_STOP = 0.0;

    private final int SLIDE_POSITION_MIN = 0;
    private final int SLIDE_POSITION_MAX = 6000;
    private final int SLIDE_POSITION_DELTA = 500;

    private DcMotorEx slideMotor;
    private int slidePosition = SLIDE_POSITION_MIN;

    public IntakeSlide(OpMode opMode){
        slideMotor = opMode.hardwareMap.get(DcMotorEx.class,HardwareConstant.SlideMotor);
        initSlide();
    }

    public void resetSlide() {
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

      // Starting position
    public void initSlide() {
        slideMotor.setPositionPIDFCoefficients(4.0);
        slideMotor.setDirection(DcMotorEx.Direction.FORWARD);
        slideMotor.setPower(POWER_LEVEL_STOP);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetSlide();
    }

    private void runMotors(boolean overideMinValue) {
        // Do not let slide go below minimum position and above maximum position
        if (!overideMinValue && slidePosition < SLIDE_POSITION_MIN) {
            slidePosition = SLIDE_POSITION_MIN;
        } else if (slidePosition >= SLIDE_POSITION_MAX) {
            slidePosition = SLIDE_POSITION_MAX;
        }

        slideMotor.setTargetPosition(slidePosition);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(POWER_LEVEL_RUN);

        if ( overideMinValue)
        {
            while (slideMotor.isBusy()) {
            }

            slideMotor.setPower(POWER_LEVEL_STOP);
        }

        if (slidePosition <= SLIDE_POSITION_MIN) {
            slidePosition = SLIDE_POSITION_MIN;
            resetSlide();
        }

        if (slidePosition >= SLIDE_POSITION_MAX) {
            slidePosition = SLIDE_POSITION_MAX;
        }
    }

    public void moveSlideHigh() {
        slidePosition = SLIDE_POSITION_MAX;
        runMotors(false);
    }
    public void moveSlideLow() {
        slidePosition = SLIDE_POSITION_MIN;
        runMotors(false);
    }

    // Sets the intake arm to a position that allows for intake
    public void extendSlide() {
        slidePosition = slidePosition + SLIDE_POSITION_DELTA;
        runMotors(false);
    }
    public void retractSlide(boolean overideMinValue) {
        slidePosition = slidePosition - SLIDE_POSITION_DELTA;
        runMotors(overideMinValue);
    }

    public int getTargetPosition() {
        return slidePosition;
    }

    public int getMotorPosition() {
        return slideMotor.getCurrentPosition();
    }
}
