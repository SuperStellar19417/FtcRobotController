package org.firstinspires.ftc.teamcode.OpModes.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;

public class ArmMoveToHighBasket implements Action {

    private Arm arm;
    private boolean initialized = false;

    private static int armMaxForAuto = 1150;
    private Telemetry telemetry;

    public ArmMoveToHighBasket(Arm arm, Telemetry telemetry){
        this.arm = arm;
        this.telemetry = telemetry;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        // powers on motor, if it is not on
        if (!initialized) {
            arm.moveArmHighBasketPosition();
            initialized = true;
        }

        // checks arm and slide current position
        double pos = arm.armMotor.getCurrentPosition();
        telemetry.addData("Arm pos:", pos);
        telemetry.update();

        if (pos < armMaxForAuto-75) {
            // true causes the action to rerun
            return true;
        } else {
            telemetry.addLine("ARM reached Low Basket");
            telemetry.update();
            // false stops action rerun
            return false;
        }
        // overall, the action powers the arm until it surpasses
        // ARM_POSITION_TICKS_LOW_BASKET encoder ticks, then powers it off

    }
}
