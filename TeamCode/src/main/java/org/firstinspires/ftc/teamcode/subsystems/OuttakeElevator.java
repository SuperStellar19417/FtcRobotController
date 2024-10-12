package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeElevator {

    public DcMotorEx outtakeMotorRight;
    public DcMotorEx outtakeMotorLeft;

    public static int ELEVATOR_POSITION_INTAKE_COUNT = 0;
    public static int ELEVATOR_POSITION_LOW_BUCKET_COUNT = 1150;
    public static int ELEVATOR_POSITION_HIGH_BUCKET_COUNT = 1450;
    public static int ELEVATOR_POSITION_LOW_RUNG_COUNT = 2050;
    public static int ELEVATOR_POSITION_HIGH_RUNG_COUNT = 2050;
    public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 150;
    public static double POWER_LEVEL_RUN = .8;
    public double motorPowerToRun = POWER_LEVEL_RUN;
    public boolean runElevatorToLevelState = false;
    public boolean elevatorNeedsToGoDown = false;

    public enum ELEVATOR_POSITION {
        ELEVATOR_POSITION_INTAKE,
        ELEVATOR_POSITION_LOW_BUCKET,
        ELEVATOR_POSITION_HIGH_BUCKET,
        ELEVATOR_POSITION_HIGH_RUNG,
        ELEVATOR_POSITION_LOW_RUNG
    }

    public ELEVATOR_POSITION elevatorPosition = ELEVATOR_POSITION.ELEVATOR_POSITION_INTAKE;
    public int elevatorPositionCount = ELEVATOR_POSITION_INTAKE_COUNT;
    public Telemetry telemetry;

    public OuttakeElevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "leftOuttake");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "rightOuttake");
        initElevator();
    }

    public void initElevator() {
        outtakeMotorLeft.setPositionPIDFCoefficients(5.0);
        outtakeMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        outtakeMotorRight.setPositionPIDFCoefficients(5.0);
        outtakeMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        resetElevator();
    }

    public void runElevatorToLevel(double power) {
        if (elevatorNeedsToGoDown) {
            elevatorPosition = ELEVATOR_POSITION.ELEVATOR_POSITION_INTAKE;
            elevatorNeedsToGoDown = false;
            resetElevator();
            stopMotors();
            return;
        }

        if (runElevatorToLevelState) {
            runMotors(power);
            runElevatorToLevelState = false;
        } else {
            stopMotors();
        }
    }

    public void resetElevator() {
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runMotors(double power) {
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotorLeft.setPower(power);
        outtakeMotorRight.setPower(power);
    }

    private void stopMotors() {
        outtakeMotorRight.setPower(0.0);
        outtakeMotorLeft.setPower(0.0);
    }
    /**
     * Initialize elevator to low position
     */
    public void moveElevatorIntakePosition() {
        elevatorPositionCount = ELEVATOR_POSITION_INTAKE_COUNT;
        outtakeMotorLeft.setTargetPosition(elevatorPositionCount);
        outtakeMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.ELEVATOR_POSITION_INTAKE;
        if (runElevatorToLevelState) {
            runElevatorToLevel(motorPowerToRun);
        }
    }

    public void moveElevatorLowBucketPosition() {
        elevatorPositionCount = ELEVATOR_POSITION_LOW_BUCKET_COUNT;
        outtakeMotorLeft.setTargetPosition(elevatorPositionCount);
        outtakeMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.ELEVATOR_POSITION_LOW_BUCKET;
        if (runElevatorToLevelState) {
            runElevatorToLevel(motorPowerToRun);
        }
    }

    public void moveElevatorHighBucketPosition() {
        elevatorPositionCount = ELEVATOR_POSITION_HIGH_BUCKET_COUNT;
        outtakeMotorLeft.setTargetPosition(elevatorPositionCount);
        outtakeMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.ELEVATOR_POSITION_HIGH_BUCKET;
        if (runElevatorToLevelState) {
            runElevatorToLevel(motorPowerToRun);
        }
    }

    public void moveElevatorLowRungPosition() {
        elevatorPositionCount = ELEVATOR_POSITION_LOW_RUNG_COUNT;
        outtakeMotorLeft.setTargetPosition(elevatorPositionCount);
        outtakeMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.ELEVATOR_POSITION_LOW_RUNG;
        if (runElevatorToLevelState) {
            runElevatorToLevel(motorPowerToRun);
        }
    }

    public void moveElevatorHighRungPosition() {
        elevatorPositionCount = ELEVATOR_POSITION_HIGH_RUNG_COUNT;
        outtakeMotorLeft.setTargetPosition(elevatorPositionCount);
        outtakeMotorRight.setTargetPosition(elevatorPositionCount);
        motorPowerToRun = POWER_LEVEL_RUN;
        runElevatorToLevelState = true;
        elevatorPosition = ELEVATOR_POSITION.ELEVATOR_POSITION_HIGH_RUNG;
        if (runElevatorToLevelState) {
            runElevatorToLevel(motorPowerToRun);
        }
    }

    public ELEVATOR_POSITION getElevatorPosition() {
        return elevatorPosition;
    }

    /**
     * Return elevator current position
     */
    public int currentLeftEncoderValue() {
        return outtakeMotorLeft.getCurrentPosition();
    }

    public int currentRightEncoderValue() {
        return outtakeMotorRight.getCurrentPosition();
    }


    public void printDebugMessages(){
        //******  debug ******
        telemetry.addData("elevator level", getElevatorPosition().toString());
        telemetry.addData("elevator_motor_encoder_left", currentLeftEncoderValue());
        telemetry.addData("elevator_motor_encoder_right",currentRightEncoderValue());
        telemetry.addData("elevator_motor_encoder val ", elevatorPositionCount);
    }



}
