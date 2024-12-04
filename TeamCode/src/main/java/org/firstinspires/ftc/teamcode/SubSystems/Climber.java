package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Climber {
    private DcMotorEx climberMotor;

    private final int CLIMBER_POSITION_MIN = 0;
    private final int CLIMBER_POSITION_MAX = 20000;

    private final int CLIMBER_POSITION_UP = CLIMBER_POSITION_MAX; // 2024-11-09 calibrated value
    private final int CLIMBER_POSITION_DOWN = 0;
    private final int CLIMBER_POSITION_DELTA = 7500;  // 2024-11-09  calibrated value
    private final double POWER_LEVEL_RUN = .9;
    private final double POWER_LEVEL_STOP = 0.0;
    private final double MAX_VELOCITY = 2680;


    private int climberMotorPosition = CLIMBER_POSITION_DOWN;


    public Climber(LinearOpMode opMode) {
        climberMotor = opMode.hardwareMap.get(DcMotorEx.class, "climberMotor");
        initClimber();
    }

    public void initClimber() {
        climberMotor.setDirection(DcMotorEx.Direction.FORWARD);
        climberMotor.setPower(POWER_LEVEL_STOP);
        climberMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        resetClimber();
    }

    public void resetClimber() {
        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopMotors() {
        climberMotor.setPower(POWER_LEVEL_STOP);
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

        climberMotor.setTargetPosition(climberMotorPosition);
        climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(POWER_LEVEL_RUN);

        while (climberMotor.isBusy()) {
            // Wait for the motor to reach the target position
        }

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

    public void moveClimberUp() {
        climberMotorPosition = CLIMBER_POSITION_UP;
        runClimberToLevel();
    }

    public void runClimberDown() {
        climberMotorPosition = CLIMBER_POSITION_DOWN;
        runClimberToLevel();
        resetClimber();
    }

    public int getClimberTargetPosition() {
        return climberMotorPosition;
    }

    public int getClimberMotorPosition() {
        return climberMotor.getCurrentPosition();
    }
}