package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Climber {
    public DcMotorEx climberMotor;
    public TouchSensor touchSensor;

    public enum CLIMBER_MOTOR_STATE {
        CLIMBER_MOTOR_ASCENT_UP,
        CLIMBER_MOTOR_ASCENT_DOWN,
        CLIMBER_MOTOR_ASCENT_SLIGHTLY_DOWN,
        CLIMBER_MOTOR_ASCENT_SLIGHTLY_UP;


    }

    public static double ENCODER_VAlUE = 0;
    public static int CLIMBER_MOTOR_ASCENT_UP_POSITION_COUNT = 0; // 2023-12-21 calibrated value
    public static int CLIMBER_MOTOR_ASCENT_DOWN_POSITION_COUNT = 0;
    public static int CLIMBER_MOTOR_ASCENT_SLIGHTLY_DOWN_POSITION_COUNT = 0;  // 2023-12-21 calibrated value
    public static int CLIMBER_MOTOR_ASCENT_SLIGHTLY_UP_POSITION_COUNT = 0;

    public static int CLIMBER_DELTA_SLIGHTLY_UP_DELTA_COUNT = 0;
    public static int CLIMBER_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = 0;

    public  CLIMBER_MOTOR_STATE climberMotorState = CLIMBER_MOTOR_STATE.CLIMBER_MOTOR_ASCENT_DOWN;
    public int climberMotorStateCount =   CLIMBER_MOTOR_ASCENT_DOWN_POSITION_COUNT;

    public static double POWER_LEVEL_RUN = 0;

    public double motorPowerToRun = POWER_LEVEL_RUN;

    public boolean climberNeedsToGoDown = false;

    public boolean isClimberInLowPosition() {
        return touchSensor.isPressed();
    }

    private void runMotors(double power) {
       climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        climberMotor.setPower(power);
    }




    public Climber(HardwareMap hardwareMap) {
        climberMotor = hardwareMap.get(DcMotorEx.class, "climberMotor");

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

    public void runCLimberToLevel(double power) {
        if (climberNeedsToGoDown) {
           // if (!isclimberInLowPosition()) {
                climberMotorStateCount = climberMotorStateCount - CLIMBER_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
                climberMotor.setTargetPosition(climberMotorStateCount);
                motorPowerToRun = POWER_LEVEL_RUN;
                runMotors(motorPowerToRun);
            } else {
                climberMotorStateCount = CLIMBER_MOTOR_ASCENT_DOWN_POSITION_COUNT;
                climberNeedsToGoDown = false;
             //   resetclimber();
              //  stopMotors();
            }
            return;
        }

    }
}

