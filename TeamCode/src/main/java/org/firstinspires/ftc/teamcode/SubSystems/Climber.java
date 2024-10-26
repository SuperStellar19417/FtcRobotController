package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Climber {
    public DcMotorEx climberMotor;


    public enum CLIMBER_MOTOR_STATE {
        CLIMBER_UP_POSITION,
        CLIMBER_DOWN_POSITION,

        CLIMBER_SLIGHTLY_DOWN_POSITION,

        CLIMBER_SLIGHTLY_UP_POSITION,


    }

    public static double ENCODER_VAlUE = 0;
    public static int CLIMBER_UP_POSITION_COUNT = 0; // 2023-12-21 calibrated value
    public static int CLIMBER_DOWN_POSITION_COUNT = 0;
    public static int CLIMBER_SLIGHTLY_DOWN_POSITION_COUNT = 0;  // 2023-12-21 calibrated value
    public static int CLIMBER_SLIGHTLY_UP_POSITION_COUNT = 0;

    public static int CLIMBER_DELTA_SLIGHTLY_UP_DELTA_COUNT = 0;
    public static int CLIMBER_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 0;

    public CLIMBER_MOTOR_STATE climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_DOWN_POSITION;
    public int climberMotorStateCount = CLIMBER_DOWN_POSITION_COUNT;

    public static double POWER_LEVEL_RUN = 0;


    public double motorPowerToRun = POWER_LEVEL_RUN;

    public boolean climberNeedsToGoDown = false;

    //public boolean isClimberInLowPosition() {
    // return touchSensor.isPressed();
    //  }

    private void runMotors(double power) {
        climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(power);
    }


    public Climber(LinearOpMode opMode) {
        climberMotor = opMode.hardwareMap.get(DcMotorEx.class, "climberMotor");
        initClimber();
    }

    public void initClimber() {
        //climberMotorLeft.setPositionPIDFCoefficients(5.0);
        climberMotor.setDirection(DcMotorEx.Direction.FORWARD);
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        resetClimber();
        //initializeClimberToLowPosition();


    }

    public void resetClimber() {

        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopMotors() {
        climberMotor.setPower(0.0);
    }

    public void runCLimberToLevel(double power, int position) {
        climberMotor.setTargetPosition(position);
        runMotors(power);

    }

    public void moveClimberSlightlyDown() {
        turnClimberBrakeModeOn();
        climberMotorStateCount = climberMotorStateCount - CLIMBER_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
        if (climberMotorStateCount < 0) {
            climberMotorStateCount = 0;
        }
    }

        public void moveClimberSlightlyUp() {
            turnClimberBrakeModeOn();
            climberMotorStateCount = climberMotorStateCount + CLIMBER_DELTA_SLIGHTLY_UP_DELTA_COUNT;
            if (climberMotorStateCount > 0) {
                climberMotorStateCount = 1000;
            }
    }



    private void turnClimberBrakeModeOn() {
        climberMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    }