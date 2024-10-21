package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    public DcMotorEx armMotor;
    public VoltageSensor voltageSensor;
    public double voltageThreshold = 9.0;
    public static int ARM_POSITION_INTAKE_COUNT = 0;
    public static int ARM_POSITION_LOW_BUCKET_COUNT = 1150;
    public static int ARM_POSITION_HIGH_BUCKET_COUNT = 1450;
    public static int ARM_POSITION_LOW_RUNG_COUNT = 2050;
    public static int ARM_POSITION_HIGH_RUNG_COUNT = 2050;
    public static int ARM_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 150;
    public static double POWER_LEVEL_RUN = .8;
    public double motorPowerToRun = POWER_LEVEL_RUN;
    public boolean runArmToLevelState = false;
    public boolean armNeedsToGoDown = false;

    public enum ARM_POSITION {
        ARM_POSITION_INTAKE,
        ARM_POSITION_LOW_BUCKET,
        ARM_POSITION_HIGH_BUCKET,
        ARM_POSITION_HIGH_RUNG,
        ARM_POSITION_LOW_RUNG
    }

    public ARM_POSITION armPosition = ARM_POSITION.ARM_POSITION_INTAKE;
    public int armPositionCount = ARM_POSITION_INTAKE_COUNT;
    public Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armMotor = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        initArm();
    }

    public void initArm() {
        armMotor.setPositionPIDFCoefficients(5.0);
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        resetArm();
    }

    public void runArmToLevel(double power) {
        if (armNeedsToGoDown) {
            armPosition = ARM_POSITION.ARM_POSITION_INTAKE;
            armNeedsToGoDown = false;
            resetArm();
            stopMotors();
            return;
        }

        if (runArmToLevelState) {
            runMotors(power);
            runArmToLevelState = false;
        } else {
            stopMotors();
        }
    }

    public void resetArm() {
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runMotors(double power) {
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setPower(power);
    }

    private void stopMotors() {
        armMotor.setPower(0.0);
    }
    /**
     * Initialize arm to low position
     */
    public void moveElevatorIntakePosition() {
        armPositionCount = ARM_POSITION_INTAKE_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveArmLowBucketPosition() {
        armPositionCount = ARM_POSITION_LOW_BUCKET_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_LOW_BUCKET;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveElevatorHighBucketPosition() {
        armPositionCount = ARM_POSITION_HIGH_BUCKET_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_HIGH_BUCKET;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveElevatorLowRungPosition() {
        armPositionCount = ARM_POSITION_LOW_RUNG_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_LOW_RUNG;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveElevatorHighRungPosition() {
        armPositionCount = ARM_POSITION_HIGH_RUNG_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runArmToLevelState = true;
        armPosition = ARM_POSITION.ARM_POSITION_HIGH_RUNG;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public ARM_POSITION getElevatorPosition() {
        return armPosition;
    }

    /**
     * Return elevator current position
     */

    public int currentArmEncoderValue() {
        return armMotor.getCurrentPosition();
    }


    public void printDebugMessages(){
        //******  debug ******
        telemetry.addData("arm level", getElevatorPosition().toString());
        telemetry.addData("arm_motor_encoder_right",currentArmEncoderValue());
        telemetry.addData("arm_motor_encoder val ", armPositionCount);
    }



}
