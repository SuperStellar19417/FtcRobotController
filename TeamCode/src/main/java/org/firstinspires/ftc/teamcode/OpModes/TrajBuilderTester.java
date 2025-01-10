package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;

@Autonomous (name = "Scoring Auto 1st Ascent TEST", group = "01-Test")
public class TrajBuilderTester extends LinearOpMode {
    private GamepadController gamepadController;
    private DriveTrain driveTrain;

    // We can transfer this from last autonomous op mode if needed,
    // but most the time we don't need to.
  //  private final Pose2d startPose = new Pose2d(-59.7, 11.16,  Math.toRadians(0));
    private final Pose2d startPose = new Pose2d(0, 0,  Math.toRadians(0));

    private final Vector2d dropPose = new Vector2d(9, 36.5); //132
    private final Vector2d dropPoseAdjust = new Vector2d(11, 35); //132
    private final Vector2d midPoseCycle = new Vector2d(17, 35); //90
    private final Vector2d cycle1 = new Vector2d(29.5, 35 ); //50
    private final Vector2d cycle2 = new Vector2d(33.5, 44); //40
    private final Vector2d midPoseSub = new Vector2d(55, 47); //90
    private final Vector2d parkPose = new Vector2d(68, 15); //90



    private Claw claw;
    private Arm arm;
    private IntakeSlide slide;
    private Climber climber;

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
        TrajectoryActionBuilder toBasket = driveTrain.actionBuilder(startPose)
                .splineTo(dropPose, Math.toRadians(132));
        //new Vector2d(5,35.5), Math.toRadians(125));
        TrajectoryActionBuilder cyclePartOne = driveTrain.actionBuilder(new Pose2d(dropPose, Math.toRadians(132)))
                .strafeToLinearHeading(midPoseCycle, Math.toRadians(55))
                .strafeToLinearHeading(cycle1, Math.toRadians(55));

        TrajectoryActionBuilder toBasket2 = driveTrain.actionBuilder(new Pose2d(cycle1, Math.toRadians(55)))
                .splineTo(dropPose, Math.toRadians(132));

        TrajectoryActionBuilder cyclePartTwo = driveTrain.actionBuilder(new Pose2d(dropPose, Math.toRadians(132)))
                .strafeToLinearHeading(midPoseCycle, Math.toRadians(45))
                .strafeToLinearHeading(cycle2, Math.toRadians(90));

        TrajectoryActionBuilder toBasket3 = driveTrain.actionBuilder(new Pose2d(cycle2, Math.toRadians(90)))
                .splineTo(dropPoseAdjust, Math.toRadians(130));


        TrajectoryActionBuilder toSub = driveTrain.actionBuilder(new Pose2d(dropPoseAdjust, Math.toRadians(130)))
                .strafeToLinearHeading(midPoseSub, Math.toRadians(100))
                .strafeToLinearHeading(parkPose, Math.toRadians(100));



        // Create an action that will be run
        Action basketAction = toBasket.build();
        Action submersibleAction = toSub.build();
        Action cycle1Action = cyclePartOne.build();
        Action basket2Action = toBasket2.build();
        Action cycle2Action = cyclePartTwo.build();
        Action basket3Action = toBasket3.build();


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
        arm.moveArmLowBucketPosition();
        Actions.runBlocking(new SequentialAction(basketAction));
        cycleToBasket();
        Actions.runBlocking(new SequentialAction(cycle1Action));
        claw.wristMid();
        claw.intakeClawClose();
        safeWaitSeconds(1);


        Actions.runBlocking(new SequentialAction(basket2Action));
        arm.moveArmLowBucketPosition();
        slide.extendSlide();
        safeWaitSeconds(1);
        claw.wristMid();
        claw.intakeClawOpen();
        safeWaitSeconds(1);

        Actions.runBlocking(cycle2Action);
    //    arm.moveArmIntakePosition();
        safeWaitSeconds(0.5);
   //     claw.intakeClawClose();

        Actions.runBlocking(basket3Action);
    //    arm.moveArmLowBucketPosition();
        safeWaitSeconds(1);
    //    claw.intakeClawOpen();
        safeWaitSeconds(0.5);

        Actions.runBlocking(new SequentialAction(submersibleAction));
     //   slide.retractSlide(false);
     //   arm.moveArmIntakePosition();
        safeWaitSeconds(1);
     //   climber.runMotorToPosition(700);
        //climber.moveClimberUp();

    }

    private void cycleToBasket() {
        slide.extendSlide();
        claw.wristMid();
        safeWaitSeconds(1.5);
        claw.intakeClawOpen();
        safeWaitSeconds(1);
        claw.wristUp();
        slide.retractSlide(false);
        safeWaitSeconds(0.5);
        arm.moveArmIntakePosition();
    }

    private void initSubsystems() {
        // Initialize all subsystems here
        telemetry.setAutoClear(false);

        // Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        // Initialize drive train
        driveTrain = new DriveTrain(hardwareMap, startPose, this);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();
        arm = new Arm(this);
        claw = new Claw(this);
        climber = new Climber(this);
        slide = new IntakeSlide(this);

        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, claw, arm, null, null);
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
        telemetry.addLine("Running Normal TeleOp Mode");

        // Output telemetry messages for subsystems here
        driveTrain.outputTelemetry();

        telemetry.update();
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }



}
