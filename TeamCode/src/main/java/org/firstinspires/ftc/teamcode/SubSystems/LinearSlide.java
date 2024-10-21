package org.firstinspires.ftc.teamcode.SubSystems;
//TODO cleanup
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    public DcMotorEx slideMotor;
    public Telemetry telemetry;

    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        slideMotor = hardwareMap.get(DcMotorEx.class,HardwareConstant.SlideMotor);

        initLinearSlide();
    }

    public enum SLIDE_MOTOR_STATE {

        SLIDE_HOLD,

        SLIDE_EXTEND,

    }

    public SLIDE_MOTOR_STATE intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_HOLD;

    // Starting position
    public void initLinearSlide() {
        slideMotor.setTargetPosition(0);
        intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_HOLD;
    }

    // Sets the intake arm to a position that allows for intake
    public void setSlidePositionHold() {
        slideMotor.setTargetPosition(0);
        intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_HOLD;
    }
    public void setSlidePositionExtend() {
        slideMotor.setTargetPosition(1000);
        intakeArmServoState = SLIDE_MOTOR_STATE.SLIDE_EXTEND;
    }

}
