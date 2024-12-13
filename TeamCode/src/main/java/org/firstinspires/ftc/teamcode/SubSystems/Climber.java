package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    public DcMotorEx climberMotor;
    public TouchSensor climberLimitSwitch;

    public final int CLIMBER_POSITION_MIN = 10;
    public final int CLIMBER_POSITION_MAX = 11000;

    public final int CLIMBER_POSITION_UP = CLIMBER_POSITION_MAX; // 2024-11-09 calibrated value
    public final int CLIMBER_POSITION_DOWN = 0;
    private final int CLIMBER_POSITION_DELTA = 2750;  // 2024-11-09  calibrated value
    private final int OVERRIDE_CLIMBER_POSITION_DELTA = 500;
    private final double POWER_LEVEL_RUN = .9;
    private final double POWER_LEVEL_STOP = 0.0;
    private final double MAX_VELOCITY = 2680;


    public int climberMotorPosition = CLIMBER_POSITION_DOWN;


    public Climber(LinearOpMode opMode) {
        climberMotor = opMode.hardwareMap.get(DcMotorEx.class, HardwareConstant.ClimberMotor);
        climberLimitSwitch = opMode.hardwareMap.get(TouchSensor.class, HardwareConstant.ClimberLimitSwitch);
        initClimber();
    }

    public void initClimber() {

        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climberMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        climberMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Calculate PIDF values for velocity control
        double kF = 32767/MAX_VELOCITY;
        double kP = 0.1 * kF;
        double kI = 0.1 * kP;
        double kD = 0.0;

        climberMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        climberMotor.setPositionPIDFCoefficients(5.0);
        resetClimber();
    }

    public void resetClimber() {
        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopMotors() {
        climberMotor.setPower(POWER_LEVEL_STOP);
        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runClimberToLevel() {
        runClimberToLevel(false);
    }

    public void runClimberToLevel(boolean overideMinValue) {

        // Do not let climber go below minimum position and above maximum position
        if (!overideMinValue && climberMotorPosition < CLIMBER_POSITION_MIN) {
            climberMotorPosition = CLIMBER_POSITION_MIN;
        } else if (climberMotorPosition >= CLIMBER_POSITION_MAX) {
            climberMotorPosition = CLIMBER_POSITION_MAX;
        }

        runMotorToPosition(climberMotorPosition);

        if (climberMotorPosition <= CLIMBER_POSITION_MIN) {
            climberMotorPosition = CLIMBER_POSITION_MIN;
            resetClimber();
        }

        if (climberMotorPosition >= CLIMBER_POSITION_MAX) {
            climberMotorPosition = CLIMBER_POSITION_MAX;
        }
    }

    public void moveClimberSlightlyDown(boolean overideMinValue) {
        climberMotorPosition = climberMotorPosition - CLIMBER_POSITION_DELTA;
        runClimberToLevel(overideMinValue);
    }

    public void moveClimberSlightlyUp() {
        climberMotorPosition = climberMotorPosition + CLIMBER_POSITION_DELTA;
        runClimberToLevel();
    }

    public boolean runMotorAllTheWayDown() {
        // Move the motor down until the limit switch is pressed
        // Here we are switching to power mode instead of velocity mode
        // because we really don't care about PID in this override case - just bring the motor down
        // also, LEDs are cool, but we are not using them here
        boolean isLimitSwitchPressed = climberLimitSwitch.isPressed();


        // If limit switch is pressed, stop the motor and return false to get out of override mode
        if (isLimitSwitchPressed) {
            stopMotor();
            return false;
        }

        // otherwise, keep moving the motor down by delta
        int position = climberMotor.getCurrentPosition();
        position -= OVERRIDE_CLIMBER_POSITION_DELTA;
        climberMotor.setTargetPosition(position);
        climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(POWER_LEVEL_RUN);

        // return true to indicate that we have not hit the limit switch
        return true;
    }

    public void runMotorToPosition(int position) {
        climberMotor.setTargetPosition(position);
        climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climberMotor.setVelocity(MAX_VELOCITY);
    }

    private void stopMotor() {
        climberMotor.setPower(POWER_LEVEL_STOP);
        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveClimberUp() {
        climberMotorPosition = CLIMBER_POSITION_UP;
        runClimberToLevel();
    }

    /*public void runClimberDown() {
        climberMotorPosition = CLIMBER_POSITION_DOWN;
        runClimberToLevel();
        resetClimber();
    } */

    public void stopClimberMotor() {
        // TODO:  reading as pressing in gamepadcontroller so add a boolean thing to toggle pressed ONCE god bless
        climberMotor.setPower(0);
        climberMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public int getClimberTargetPosition() {
        return climberMotorPosition;
    }

    public int getClimberMotorPosition() {
        return climberMotor.getCurrentPosition();
    }
}