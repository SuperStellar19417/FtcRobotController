package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

@Autonomous(name = "Simple Path Test Auto", group = "01-Test")
public class SimplePathTestAuto extends LinearOpMode {

    private GamepadController gamepadController;
    private DriveTrain driveTrain;

    // We can tranfer this from last autonoumous opmode if needed,
    // but most the time we don't need to.
    private Pose2d startPose = new Pose2d(0, 0,  Math.toRadians(0));

    @Override
    public void runOpMode() {
        // See https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/ for more information on how to create a path
        // Initialization code here
        initSubsystems();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        // Create a simple path here
        TrajectoryActionBuilder tab1 = driveTrain.actionBuilder(startPose)
                .lineToX(14) // Move forward 14 inches
                .turn(Math.toRadians(90))// Turn 90 degrees
                .lineToY(6); // Move6 inches another

        Action followPathAction = tab1.build();

        Actions.runBlocking( new SequentialAction(followPathAction));

    }

    private void initSubsystems() {
        // Initialize all subsystems here
        telemetry.setAutoClear(false);

        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Intialize drive train
        driveTrain = new DriveTrain(hardwareMap, startPose, this);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        // Set the bulk mode to auto for control and expansion hubs
        // This optimizes the communication between the robot controller and the expansion hubs and
        // motors, sensors, etc. connected to them.
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Robot Init Completed");
        telemetry.addLine("====================");
        telemetry.update();
    }

    /**
     * Output telemetry messages to the driver station
     */
    public void outputTelemetry(){

        telemetry.setAutoClear(true);
        telemetry.addLine("Running Normal TeleOpMode");

        // Output telemetry messages for susbsystems here
        driveTrain.outputTelemetry();

        telemetry.update();
    }
}
