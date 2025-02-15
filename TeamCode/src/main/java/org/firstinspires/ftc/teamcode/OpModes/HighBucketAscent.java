package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Flag;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.SubSystems.SampleColorLight;
import org.firstinspires.ftc.teamcode.Utils;


@Autonomous(name = "High Bucket Ascent", group = "01-Test")
public class HighBucketAscent extends LinearOpMode {

    private GamepadController gamepadController;
    private MecanumDrive driveTrain;

    // We can transfer this from last autonomous op mode if needed,
    // but most the time we don't need to.
    private final Pose2d startPose = new Pose2d(0, 0,  Math.toRadians(0));

    private Claw claw;
    private Arm arm;
    private Flag flag;
    private SampleColorLight sampleColorLight;

    private IntakeSlide linearSlide;

    @Override
    public void runOpMode() {
        // See https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
        // for more information on how to create a path

        // Initialization code here
        initSubsystems();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        // Create a simple path here
        // We are using RoadRunner's TrajectoryBuilder to create a simple path with a 0,0,0 start pose
       // TrajectoryActionBuilder tab1 = driveTrain.actionBuilder(startPose)
               // .turn(Math.toRadians(-90))
               // .lineToX(5);
               // arm.runArmToLevel(5)
                //.turn(Math.toRadians(90))
                        //.lineToX(10)
               // .turn(Math.toRadians(90))
                //.lineToX(10);
               // arm.runArmToLevel(5);




        // Create an action that will be run
        //Action followPathAction = tab1.build();

        // Run the action (s)
        // You can run multiple actions to execute a complex auto. For example :
        /*
            Actions.runBlocking(
            new SequentialAction(
                    trajectoryActionChosen,
                    lift.liftUp(),
                    claw.openClaw(),
                    lift.liftDown(),
                    trajectoryActionCloseOut
            ));
        */
        // TrajectoryActionBuilder creates the path you want to follow and actions are subsystem actions
        // that should be executed once that path is completed.
       // Actions.runBlocking( new SequentialAction(followPathAction));

    }

    private void initSubsystems() {
        // Initialize all subsystems here
        telemetry.setAutoClear(false);


        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Initialize drive train
        driveTrain = new MecanumDrive(hardwareMap, startPose);
        telemetry.addData("DriveTrain Initialized with Pose:", Utils.toStringPose2d(startPose));
        telemetry.update();

        //Aarushi-initialize claw and arm
       arm = new Arm( this);
        telemetry.addLine("Arm initialized");
       claw = new Claw(this);
       linearSlide = new IntakeSlide( this);
        flag = new Flag(this);
        sampleColorLight = new SampleColorLight(this);

        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, claw, arm, linearSlide, null, flag, sampleColorLight);

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
        telemetry.addLine("Running High Bucket Ascent");

        // Output telemetry messages for subsystems here
        Utils.outputDriveTelemetry(telemetry, gamepadController.driveType, driveTrain);
        telemetry.update();
    }
}

