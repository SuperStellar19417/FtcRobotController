package org.firstinspires.ftc.teamcode.SubSystems;
//TODO cleanup
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    public DcMotorEx slideMotor;
    public Telemetry telemetry;

    public LinearSlide(OpMode opMode){
        slideMotor = opMode.hardwareMap.get(DcMotorEx.class,HardwareConstant.SlideMotor);
        initLinearSlide();
    }

    public enum SLIDE_MOTOR_STATE {

        SLIDE_HOLD,

        SLIDE_EXTEND,

    }
    public int slidePositionCount = 0;

    public SLIDE_MOTOR_STATE intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_HOLD;

    // Starting position
    public void initLinearSlide() {
        slideMotor.setTargetPosition(0);
        slideMotor.setPositionPIDFCoefficients(4.0);
        slideMotor.setDirection(DcMotorEx.Direction.FORWARD);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_HOLD;
    }

    private void runMotors() {

        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(slidePositionCount);
        slideMotor.setPower(0.5);
    }

    // Sets the intake arm to a position that allows for intake
    public void setSlidePositionHold() {
        slidePositionCount = 0;
        runMotors();
        intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_HOLD;
    }
    public void setSlidePositionExtend() {
        slidePositionCount = 2900;
        runMotors();
        intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_EXTEND;
    }

}
