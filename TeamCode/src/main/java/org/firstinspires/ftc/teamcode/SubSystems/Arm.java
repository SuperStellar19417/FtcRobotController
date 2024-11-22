package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm {

    public DcMotorEx armMotor;
    public static int ARM_POSITION_INTAKE_COUNT = 10;
    public static int ARM_POSITION_LOW_BUCKET_COUNT = 1000;
    public static int ARM_POSITION_HIGH_BUCKET_COUNT = 1050;
    public static int ARM_POSITION_LOW_RUNG_COUNT = 1600;
    public static int ARM_POSITION_HIGH_RUNG_COUNT = 1250;
    public static int ARM_POSITION_HANGING_COUNT = 2800;
  //  public static int ARM_MAX_POSITION_COUNT = 4250;
//    public static int ARM_MIN_POSITION_COUNT = 0;

    public static int ARM_DELTA_COUNT = 10;
    public static double POWER_LEVEL_RUN = 0.8;

    public double currentPID = 3.00;

    public Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };

    public enum ARM_POSITION {
        ARM_POSITION_INTAKE,
        ARM_POSITION_LOW_BUCKET,
        ARM_POSITION_HIGH_BUCKET,
        ARM_POSITION_HIGH_RUNG,
        ARM_POSITION_LOW_RUNG,
        ARM_POSITION_HANGING
    }

    public ARM_POSITION armPosition = ARM_POSITION.ARM_POSITION_INTAKE;
    public int armPositionCount = ARM_POSITION_INTAKE_COUNT;

    private OpMode opmode;

    public Arm(OpMode opMode) {
        this.opmode = opMode;
        armMotor = opMode.hardwareMap.get(DcMotorEx.class, HardwareConstant.ArmMotor);
        //limitSwitch = opMode.hardwareMap.get(TouchSensor.class, HardwareConstant.LimitSwitch);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setPositionPIDFCoefficients(currentPID);
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(0);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        opmode.telemetry.addData("current position", this.armMotor.getCurrentPosition());
    }

    private void runMotors() {

        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(armPositionCount);
        armMotor.setPower(POWER_LEVEL_RUN);
    }

    /**
     * Initialize arm to low position
     */
    public void moveArmIntakePosition()  {
        int check = armMotor.getCurrentPosition();
        armPositionCount = ARM_POSITION_INTAKE_COUNT;
        armPosition = ARM_POSITION.ARM_POSITION_INTAKE;
        armMotor.setTargetPosition(armPositionCount);
        runMotors();
    }

    public void moveArmHangingPosition() {
        armPositionCount = ARM_POSITION_HANGING_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        armPosition = ARM_POSITION.ARM_POSITION_HANGING;
        runMotors();
    }

    public void moveArmLowBucketPosition() {

        armPositionCount = ARM_POSITION_LOW_BUCKET_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        armPosition = ARM_POSITION.ARM_POSITION_LOW_BUCKET;
        runMotors();
    }

    public Action moveArmHighBucketPosition() {
        armPositionCount = ARM_POSITION_HIGH_BUCKET_COUNT;
        armMotor.setTargetPosition(armPositionCount);
        armPosition = ARM_POSITION.ARM_POSITION_HIGH_BUCKET;

        runMotors();
        return action;

    }

    public void moveArmLowRungPosition()  {
        armPositionCount = ARM_POSITION_LOW_RUNG_COUNT;
        armMotor.setTargetPosition(armPositionCount);

        armPosition = ARM_POSITION.ARM_POSITION_LOW_RUNG;

        runMotors();
    }

    public void moveArmHighRungPosition()  {
        armPositionCount = ARM_POSITION_HIGH_RUNG_COUNT;
        armMotor.setTargetPosition(armPositionCount);

        armPosition = ARM_POSITION.ARM_POSITION_HIGH_RUNG;

        runMotors();
    }

    public void moveArmSlightlyUp()  {
        armPositionCount = armPositionCount + ARM_DELTA_COUNT;

     /*   if (armPositionCount >= ARM_MAX_POSITION_COUNT) {
            armPositionCount = ARM_MAX_POSITION_COUNT;
        } */

        armMotor.setTargetPosition(armPositionCount);

        runMotors();
    }

    public void moveArmSlightlyDown()  {
        armPositionCount = armPositionCount - ARM_DELTA_COUNT;
        armMotor.setTargetPosition(armPositionCount);

        /* if (armPositionCount <= ARM_MIN_POSITION_COUNT) {
            armPositionCount = ARM_MIN_POSITION_COUNT;
        } */

        runMotors();
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
        opmode.telemetry.addData("arm level", getArmPosition().toString());
        opmode.telemetry.addData("arm_motor_encoder_right",currentArmEncoderValue());
        opmode.telemetry.addData("arm_motor_encoder val ", armPositionCount);
    }

}
