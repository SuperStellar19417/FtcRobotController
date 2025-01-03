package org.firstinspires.ftc.teamcode.SubSystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class IntakeSlide {

    private final double POWER_LEVEL_RUN = 0.9;
    private final double POWER_LEVEL_STOP = 0.0;
    private final int SLIDE_POSITION_MIN = 0;
    private final int SLIDE_POSITION_MAX = 1050;
    private final int SLIDE_POSITION_DELTA = 250;
    private final double MAX_VELOCITY = 2720*0.8;

    private DcMotorEx slideMotor;
    public TouchSensor slideLimitSwitch;
    private int slidePosition = SLIDE_POSITION_MIN;

    public IntakeSlide(OpMode opMode){
        slideMotor = opMode.hardwareMap.get(DcMotorEx.class,HardwareConstant.SlideMotor);
        slideLimitSwitch = opMode.hardwareMap.get(TouchSensor.class, HardwareConstant.SlidesLimitSwitch);
        initSlide();
    }

    public void resetSlide() {
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

      // Starting position
    public void initSlide() {
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Calculate PIDF values for velocity control
        double kF = 32767/MAX_VELOCITY;
        double kP = 0.1 * kF;
        double kI = 0.1 * kP;
        double kD = 0.0;

        slideMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        slideMotor.setPositionPIDFCoefficients(5.0);
    }

    private void runMotors() {
        // Do not let slide go below minimum position and above maximum position
        if (slidePosition < SLIDE_POSITION_MIN) {
            slidePosition = SLIDE_POSITION_MIN;
        } else if (slidePosition >= SLIDE_POSITION_MAX) {
            slidePosition = SLIDE_POSITION_MAX;
        }

        slideMotor.setTargetPosition(slidePosition);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setVelocity(MAX_VELOCITY);


    }

    public void moveSlideHigh() {
        slidePosition = SLIDE_POSITION_MAX;
        runMotors();
    }
    public void moveSlideLow() {
        slidePosition = SLIDE_POSITION_MIN;
        runMotors();
    }

    public boolean runSlideMotorAllTheWayDown() {
        boolean isLimitSwitchPressed = slideLimitSwitch.isPressed();


        // If limit switch is pressed, stop the motor and return false to get out of override mode
        if (isLimitSwitchPressed) {
            stopIntakeMotor();
            return false;
        }

        // otherwise, keep moving the motor down by delta
        int position = slideMotor.getCurrentPosition();
        position -= SLIDE_POSITION_DELTA;
        slideMotor.setTargetPosition(position);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(POWER_LEVEL_RUN);

        // return true to indicate that we have not hit the limit switch
        return true;
    }

    // Sets the intake arm to a position that allows for intake
    public void extendSlide() {
        slidePosition = slidePosition + SLIDE_POSITION_DELTA;
        runMotors();
    }
    public void retractSlide(boolean overideMinValue) {
        slidePosition = slidePosition - SLIDE_POSITION_DELTA;
        runMotors();
    }

    public void stopIntakeMotor() {
        slideMotor.setPower(0);
        slidePosition = SLIDE_POSITION_MIN;
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getTargetPosition() {
        return slidePosition;
    }

    public int getMotorPosition() {
        return slideMotor.getCurrentPosition();
    }
}
