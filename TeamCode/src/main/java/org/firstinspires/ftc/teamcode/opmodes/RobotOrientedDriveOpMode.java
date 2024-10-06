package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.SSMecanumDrive;

/**
 * SimpleRobotOrientedDriveOpMode - A simple teleop opmode for a mecanum drive train
 * OpMode for a mecanum drive train with robot oriented drive
 */
@TeleOp(name="Robot Oriented Drive OpMode", group="Tests")
public class RobotOrientedDriveOpMode extends LinearOpMode {
    private final SSMecanumDrive SSMecanumDrive = new SSMecanumDrive();

    @Override
    public void runOpMode() {

        // Initialize the mecanum drive train
        SSMecanumDrive.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Run the loop while the opmode is active
        while (opModeIsActive()) {
            // Get the gamepad inputs
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // Set the powers for the mecanum drive train
            SSMecanumDrive.drive(forward, strafe, rotate);

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}