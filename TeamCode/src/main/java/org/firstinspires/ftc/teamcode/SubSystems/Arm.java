package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    public DcMotorEx armMotor;
    public static int ARM_POSITION_INTAKE_COUNT = 0;
    public static int ARM_POSITION_LOW_BUCKET_COUNT = 1150;
    public static int ARM_POSITION_HIGH_BUCKET_COUNT = 2050;
    public static int ARM_POSITION_LOW_RUNG_COUNT = 1000;
    public static int ARM_POSITION_HIGH_RUNG_COUNT = 1900;
    public static double currentPID = 3.00;
    public static int ARM_DELTA_COUNT = 700;
    public static double POWER_LEVEL_RUN = .25;
    public double motorPowerToRun = POWER_LEVEL_RUN;
    public boolean runArmToLevelState = false;
    public boolean armNeedsToGoDown = false;
    public TouchSensor limitSwitch;
    public int prevPosition = 0;

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

    public Arm(OpMode opMode) {
        armMotor = opMode.hardwareMap.get(DcMotorEx.class, HardwareConstant.ArmMotor);
        limitSwitch = opMode.hardwareMap.get(TouchSensor.class, HardwareConstant.LimitSwitch);
        initArm();
    }

    public void initArm() {
        armMotor.setPositionPIDFCoefficients(4.0);
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
        int delta = Math.abs(armPositionCount - prevPosition);
        int checkCount = (delta - (delta%200))/200;
        for (int i = 0; i < checkCount; i++) {
            if(!limitSwitch.isPressed()) {
                prevPosition = prevPosition + delta;
                armMotor.setTargetPosition(prevPosition);
                armMotor.setPower(power);
            } else {
                return;
            }
        }
    }

    private void stopMotors() {
        armMotor.setPower(0.0);
    }
    /**
     * Initialize arm to low position
     */
    public void moveArmIntakePosition() {
        prevPosition = armPositionCount;
        armPositionCount = ARM_POSITION_INTAKE_COUNT;
        runArmSetup();
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveArmLowBucketPosition() {
        prevPosition = armPositionCount;
        armPositionCount = ARM_POSITION_LOW_BUCKET_COUNT;
        runArmSetup();
        armPosition = ARM_POSITION.ARM_POSITION_LOW_BUCKET;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveArmHighBucketPosition() {
        prevPosition = armPositionCount;
        armPositionCount = ARM_POSITION_HIGH_BUCKET_COUNT;
        runArmSetup();
        armPosition = ARM_POSITION.ARM_POSITION_HIGH_BUCKET;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveArmLowRungPosition() {
        prevPosition = armPositionCount;
        armPositionCount = ARM_POSITION_LOW_RUNG_COUNT;
        runArmSetup();
        armPosition = ARM_POSITION.ARM_POSITION_LOW_RUNG;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveArmHighRungPosition() {
        prevPosition = armPositionCount;
        armPositionCount = ARM_POSITION_HIGH_RUNG_COUNT;
        runArmSetup();
        armPosition = ARM_POSITION.ARM_POSITION_HIGH_RUNG;
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveArmSlightlyUp() {
        prevPosition = armPositionCount;
        armPositionCount = armPositionCount + ARM_DELTA_COUNT;
        runArmSetup();
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void moveArmSlightlyDown() {
        armPositionCount = armPositionCount - ARM_DELTA_COUNT;
        runArmSetup();
        if (runArmToLevelState) {
            runArmToLevel(motorPowerToRun);
        }
    }

    public void runArmSetup() {
        armMotor.setTargetPosition(armPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runArmToLevelState = true;
    }

    public ARM_POSITION getArmPosition() {
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
        telemetry.addData("arm level", getArmPosition().toString());
        telemetry.addData("arm_motor_encoder_right",currentArmEncoderValue());
        telemetry.addData("arm_motor_encoder val ", armPositionCount);
    }



}
