package org.firstinspires.ftc.teamcode.SubSystems;
//TODO cleanup
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    public DcMotorEx slideMotor;
    public Telemetry telemetry;
    private Action action = new Action() {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return true;
        }
    };

    public LinearSlide(OpMode opMode){
        slideMotor = opMode.hardwareMap.get(DcMotorEx.class,HardwareConstant.SlideMotor);
        initLinearSlide();
    }

    public enum SLIDE_MOTOR_STATE {

        SLIDE_HOLD,

        SLIDE_EXTEND,

    }
    public int slidePositionCount = 0;

    public SLIDE_MOTOR_STATE slideMotorState = SLIDE_MOTOR_STATE.SLIDE_HOLD;

    // Starting position
    public void initLinearSlide() {
        slideMotor.setTargetPosition(0);
        slideMotor.setPositionPIDFCoefficients(4.0);
        slideMotor.setDirection(DcMotorEx.Direction.FORWARD);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slideMotorState = SLIDE_MOTOR_STATE.SLIDE_HOLD;
    }

    private void runMotors() {

        slideMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideMotor.setTargetPosition(slidePositionCount);
        slideMotor.setPower(0.8);
    }

    public void moveSlideHigh() {
        slidePositionCount = 6800;
        runMotors();
        slideMotorState = SLIDE_MOTOR_STATE.SLIDE_EXTEND;
    }
    public void moveSlideLow() {
        slidePositionCount = 0;
        runMotors();
        slideMotorState = SLIDE_MOTOR_STATE.SLIDE_HOLD;
    }

    // Sets the intake arm to a position that allows for intake
    public void moveSlideUp() {
        slidePositionCount = slidePositionCount + 300;
        if (slidePositionCount >= 6600) {
            slidePositionCount = 6600;
        }
        runMotors();
        slideMotorState = SLIDE_MOTOR_STATE.SLIDE_HOLD;
    }
    public void moveSlideDown() {
        slidePositionCount = slidePositionCount - 300;
        if (slidePositionCount <= 0) {
            slidePositionCount = 0;
        }
        runMotors();
        slideMotorState = SLIDE_MOTOR_STATE.SLIDE_EXTEND;
    }

}
