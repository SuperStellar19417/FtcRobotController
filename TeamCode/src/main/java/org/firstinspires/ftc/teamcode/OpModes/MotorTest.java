package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

@TeleOp(name = "Motor Test", group = "00-Teleop")
public class MotorTest extends LinearOpMode {

    private GamepadController gamepad;
    private Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {
        arm = new Arm(this);
        gamepad = new GamepadController(gamepad1, gamepad2, null, this, null, arm, null);

        waitForStart();
        while(!isStopRequested()) {
            telemetry.update();
            while(opModeIsActive()) {
                if (gamepad.gp2GetDpad_upPress()) {
                    arm.moveArmSlightlyUp();
                    telemetry.addLine(arm.armPositionCount + " ");
                } else if (gamepad.gp2GetDpad_downPress()) {
                    arm.moveArmSlightlyDown();
                    telemetry.addLine(arm.armPositionCount + " ");
                }
            }
        }
    }
}
