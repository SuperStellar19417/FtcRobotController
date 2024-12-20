package org.firstinspires.ftc.teamcode.SubSystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Arm {

    public enum ARM_POSITION {
        ARM_POSITION_INTAKE,
        ARM_POSITION_LOW_BUCKET,
        ARM_POSITION_HIGH_BUCKET,
        ARM_POSITION_HIGH_RUNG,
        ARM_POSITION_LOW_RUNG,
        ARM_POSITION_HANGING,

        ARM_POSITION_SPECIMEN_INTAKE;
    }

    private DcMotorEx armMotor;
    private final int ARM_POSITION_TICKS_INTAKE = 150;
    private final int ARM_POSITION_TICKS_LOW_BUCKET = 1000;
    private final int ARM_POSITION_TICKS_HIGH_BUCKET = 1050;
    private final int ARM_POSITION_TICKS_LOW_RUNG = 1600;
    private final int ARM_POSITION_TICKS_HIGH_RUNG = 1250;
    private final int ARM_POSITION_TICKS_HANGING = 3000;

    private final int ARM_POSITION_SPECIMEN_INTAKE = 0;
    public static int ARM_MAX_POSITION_COUNT = 1100;
//    public static int ARM_MIN_POSITION_COUNT = 0;

    private final int ARM_DELTA_TICKS_NORMAL = 150;
    private final int ARM_DELTA_TICKS_END_GAME = 400;

    private final double POWER_LEVEL_STOP = 0.0;
    private final double POWER_LEVEL_RUN = 0.7;
    private final double POWER_LEVEL_END_GAME = 0.9;

    private double currentPowerLevel = POWER_LEVEL_RUN;
    private int currentDeltaTicks = ARM_DELTA_TICKS_NORMAL;

    private final double currentPID = 3.00;

    private ARM_POSITION currentArmPosition = ARM_POSITION.ARM_POSITION_INTAKE;
    private int armPositionTicks = ARM_POSITION_TICKS_INTAKE;

    private OpMode opmode;

    public Arm(OpMode opMode) {
        this.opmode = opMode;
        armMotor = opMode.hardwareMap.get(DcMotorEx.class, HardwareConstant.ArmMotor);
        //limitSwitch = opMode.hardwareMap.get(TouchSensor.class, HardwareConstant.LimitSwitch);
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotor.setPositionPIDFCoefficients(currentPID);
        armMotor.setDirection(DcMotorEx.Direction.FORWARD);
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setPower(POWER_LEVEL_STOP);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //moveArmIntakePosition();
    }

    public Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };

    public void setNormalMode() {
        currentPowerLevel = POWER_LEVEL_RUN;
        currentDeltaTicks = ARM_DELTA_TICKS_NORMAL;
    }

    public void setEndGameMode() {
        currentPowerLevel = POWER_LEVEL_END_GAME;
        currentDeltaTicks = ARM_DELTA_TICKS_END_GAME;
    }

    /**
     * Initialize arm to low position
     */
    public void moveArmIntakePosition()  {
        int check = armMotor.getCurrentPosition();
        armPositionTicks = ARM_POSITION_TICKS_INTAKE;
        currentArmPosition = ARM_POSITION.ARM_POSITION_INTAKE;
        armMotor.setTargetPosition(armPositionTicks);
        runMotors();
    }

    public void moveArmSpecimenIntakePosition() {
        armPositionTicks = ARM_POSITION_SPECIMEN_INTAKE;
        armMotor.setTargetPosition(armPositionTicks);
        currentArmPosition = ARM_POSITION.ARM_POSITION_SPECIMEN_INTAKE;
        runMotors();


    }

    public void moveArmHangingPosition() {
        armPositionTicks = ARM_POSITION_TICKS_HANGING;
        armMotor.setTargetPosition(armPositionTicks);
        currentArmPosition = ARM_POSITION.ARM_POSITION_HANGING;
        runMotors();
    }

    public void moveArmLowBucketPosition() {

        armPositionTicks = ARM_POSITION_TICKS_LOW_BUCKET;
        armMotor.setTargetPosition(armPositionTicks);
        currentArmPosition = ARM_POSITION.ARM_POSITION_LOW_BUCKET;
        runMotors();
    }

    public Action moveArmPlaceSpecimenPosition() {
        armPositionTicks -= 150;
        armMotor.setTargetPosition(armPositionTicks);
       // currentArmPosition = ARM_POSITION.ARM_POSITION_HIGH_BUCKET;

        runMotors();
        return action;

    }

    public void moveArmLowRungPosition()  {
        armPositionTicks = ARM_POSITION_TICKS_LOW_RUNG;
        armMotor.setTargetPosition(armPositionTicks);

        currentArmPosition = ARM_POSITION.ARM_POSITION_LOW_RUNG;

        runMotors();
    }

    public void moveArmHighRungPosition()  {
        armPositionTicks = ARM_POSITION_TICKS_HIGH_RUNG;
        armMotor.setTargetPosition(armPositionTicks);

        currentArmPosition = ARM_POSITION.ARM_POSITION_HIGH_RUNG;

        runMotors();
    }

    public void moveArmSlightlyUp()  {
        armPositionTicks = armPositionTicks + currentDeltaTicks;

        if (armPositionTicks >= ARM_MAX_POSITION_COUNT) {
            armPositionTicks = ARM_MAX_POSITION_COUNT;
        }

        armMotor.setTargetPosition(armPositionTicks);

        runMotors();
    }

    public void moveArmSlightlyDown()  {
        armPositionTicks = armPositionTicks - currentDeltaTicks;

        if (armPositionTicks <= ARM_POSITION_TICKS_INTAKE)
            armPositionTicks = ARM_POSITION_TICKS_INTAKE;

        armMotor.setTargetPosition(armPositionTicks);

        runMotors();
    }



    public ARM_POSITION getCurrentArmPosition() {
        return currentArmPosition;
    }

    public int getCurrentArmEncoderValue() {
        return armMotor.getCurrentPosition();
    }

    private void runMotors() {
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(armPositionTicks);
        armMotor.setPower(currentPowerLevel);
    }
}
