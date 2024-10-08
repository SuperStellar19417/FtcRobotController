package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeArm {
    public Servo servoIntakeArm;
    public Telemetry telemetry;

    public IntakeArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        servoIntakeArm = hardwareMap.get(Servo.class,HardwareConstant.ServoIntakeArm);

        initIntakeArm();
    }

    public enum INTAKE_ARM_SERVO_STATE {

        INTAKE_ARM_INTAKE,

        INTAKE_ARM_CARRY,

        INTAKE_ARM_TRANSFER
    }

    public INTAKE_ARM_SERVO_STATE intakeArmServoState = INTAKE_ARM_SERVO_STATE.INTAKE_ARM_INTAKE;

    // Starting position
    public void initIntakeArm() {
        servoIntakeArm.setPosition(0.00);
        intakeArmServoState = INTAKE_ARM_SERVO_STATE.INTAKE_ARM_INTAKE;
    }

    // Sets the intake arm to a position that allows for intake
    public void setIntakeArmIntakePosition() {
        servoIntakeArm.setPosition(0.00);
        intakeArmServoState = INTAKE_ARM_SERVO_STATE.INTAKE_ARM_INTAKE;
    }

    // Sets the intake arm to a position that holds the specimen/sample
    public void setIntakeArmCarryPosition() {
        servoIntakeArm.setPosition(0.00);
        intakeArmServoState = INTAKE_ARM_SERVO_STATE.INTAKE_ARM_CARRY;
    }

    // Sets the intake arm to be able to drop the specimen to the other claw
    public void setIntakeArmTransferPosition() {
        servoIntakeArm.setPosition(0.00);
        intakeArmServoState = INTAKE_ARM_SERVO_STATE.INTAKE_ARM_TRANSFER;
        // Sets the intake arm to a position
    }

}
