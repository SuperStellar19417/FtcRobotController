package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Flag;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;
import org.firstinspires.ftc.teamcode.SubSystems.SampleColorLight;
import org.firstinspires.ftc.teamcode.Utils;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "Scoring Auto 1st Ascent APRILTAG", group = "01-Test")
public class SpecimenAprilTag extends LinearOpMode {
    private GamepadController gamepadController;
    private MecanumDrive driveTrain;

    // We can transfer this from last autonomous op mode if needed,
    // but most the time we don't need to.
  //  private final Pose2d startPose = new Pose2d(-59.7, 11.16,  Math.toRadians(0));
    private Pose2d startPose; //ab -2, -60
    private final Vector2d dropPose = new Vector2d(-60, -52); //132
  //  private final Vector2d dropPoseAdjust = new Vector2d(10, 33.5); //132
   // private final Vector2d dropPoseAdjust2 = new Vector2d(17, 30);
    private final Vector2d midPoseCycle = new Vector2d(-46, -50); //90
    private final Vector2d cycle1 = new Vector2d(-60, -28); //50
    private final Vector2d cycle2 = new Vector2d(-68, -25); //40
    private final Vector2d midPoseSub = new Vector2d(-52, -45); //90
    private final Vector2d parkPose = new Vector2d(-27, -13); //90

    //private Pose2d lastPose = new Pose2d(new Vector2d(-60, 12), 90);

    private AprilTagProcessor aprilTag;




    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;



    private Claw claw;
    private Arm arm;
    private IntakeSlide slide;
    //private Climber climber;
    private Flag flag;
    private SampleColorLight sampleColorLight;

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

        //new Vector2d(5,35.5), Math.toRadians(125));



        // Create an action that will be run
      /*  Action basketAction = toBasket.build();
        Action submersibleAction = toSub.build();
        Action cycle1Action = cyclePartOne.build();
        Action basket2Action = toBasket2.build();
        Action cycle2Action = cyclePartTwo.build();
        Action basket3Action = toBasket3.build(); */


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
        //new Vector2d(5,35.5), Math.toRadians(125));



    //    arm.moveArmLowBasketPosition();
     //   slide.moveSlideMid();
        startPose = getPose();
        TrajectoryActionBuilder toBasket = driveTrain.actionBuilder(startPose)
                .splineTo(dropPose, Math.toRadians(134));
      //  telemetry.addLine(getPose().toString());
     //   telemetry.update();
        safeWaitSeconds(1);
        Actions.runBlocking(toBasket.build());
        cycleToBasket();

        TrajectoryActionBuilder midPose1 = driveTrain.actionBuilder(getPose())
                .strafeToLinearHeading(midPoseCycle, Math.toRadians(0));
        telemetry.addLine(getPose().toString());
        telemetry.update();
        safeWaitSeconds(1);
        Actions.runBlocking(midPose1.build());

        TrajectoryActionBuilder toCycle1 = driveTrain.actionBuilder(getPose())
                .strafeToLinearHeading(cycle1, Math.toRadians(0));
        Actions.runBlocking(toCycle1.build());

        claw.intakeClawOpen();
        safeWaitSeconds(.5);
        claw.wristMid();
        safeWaitSeconds(1);
        claw.intakeClawClose();
        safeWaitSeconds(1.5);
        claw.wristUp();

        arm.moveArmLowBasketPosition();
        slide.moveSlideMid();
        safeWaitSeconds(.5);

        TrajectoryActionBuilder toBasket2 = driveTrain.actionBuilder(getPose())
                .splineTo(dropPose, Math.toRadians(140));
        Actions.runBlocking(toBasket2.build());
        safeWaitSeconds(1);
        claw.wristMid();
        claw.intakeClawOpen();
        safeWaitSeconds(0.75);
        slide.moveSlideLow();

        TrajectoryActionBuilder midPose2 = driveTrain.actionBuilder(getPose())
                .strafeToLinearHeading(midPoseCycle, Math.toRadians(40));
        Actions.runBlocking(midPose2.build());

        TrajectoryActionBuilder toCycle2 = driveTrain.actionBuilder(getPose())
                .strafeToLinearHeading(cycle2, Math.toRadians(0));
        Actions.runBlocking(toCycle2.build());
        claw.intakeClawOpen();
        safeWaitSeconds(.5);
        claw.wristMid();
        safeWaitSeconds(1);
        claw.intakeClawClose();
        safeWaitSeconds(1);
        claw.wristUp();


        arm.moveArmLowBasketPosition();
        slide.moveSlideMid();

        TrajectoryActionBuilder toBasket3 = driveTrain.actionBuilder(getPose())
                .splineTo(dropPose, Math.toRadians(130));
        Actions.runBlocking(toBasket3.build());
        safeWaitSeconds(1);
        claw.intakeClawOpen();
        safeWaitSeconds(0.5);
        slide.moveSlideLow();

        TrajectoryActionBuilder midPose3 = driveTrain.actionBuilder(getPose())
                .strafeToLinearHeading(midPoseSub, Math.toRadians(100));
        Actions.runBlocking(new SequentialAction(midPose3.build()));

        TrajectoryActionBuilder toSub = driveTrain.actionBuilder(getPose())
                .strafeToLinearHeading(parkPose, Math.toRadians(100));
        Actions.runBlocking(toSub.build());
     //   slide.retractSlide(false);
     //   arm.moveArmIntakePosition();
        safeWaitSeconds(0.5);
        flag.setFlagUp();
        safeWaitSeconds(0.5);
     //   climber.runMotorToPosition(700);
        //climber.moveClimberUp();

    }

    private void cycleToBasket() {
        claw.wristMid();
        safeWaitSeconds(1);
        claw.intakeClawOpen();
        safeWaitSeconds(0.5);
        claw.wristUp();
        slide.moveSlideLow();
        safeWaitSeconds(0.75);
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
        driveTrain = new MecanumDrive(hardwareMap, startPose);
        telemetry.addData("DriveTrain Initialized with Pose:",Utils.toStringPose2d(startPose));
        telemetry.update();
        arm = new Arm(this);
        claw = new Claw(this);
   //     climber = new Climber(this);
        slide = new IntakeSlide(this);
        flag = new Flag(this);
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);


        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, this, claw, arm, null, null, flag, sampleColorLight);
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
        getPose();
        telemetry.update();
        claw.wristUp();
    }

    /**
     * Output telemetry messages to the driver station
     */
    public void outputTelemetry(){

        telemetry.setAutoClear(true);
        telemetry.addLine("Running Normal TeleOp Mode");

        // Output telemetry messages for subsystems here
        Utils.outputDriveTelemetry(telemetry, gamepadController.driveType, driveTrain);

        telemetry.update();
    }

    private Pose2d getPose() {

        double tempX = 0;

        List<AprilTagDetection> detectionList = aprilTag.getDetections();

        ArrayList<Double> xList = new ArrayList<>(50);
        for (AprilTagDetection detection : detectionList) {
            if (detection.metadata != null) {
                for (int i = 0; i < 50; i++) {
                    xList.set(i, detection.ftcPose.x);
                    tempX = detection.ftcPose.x;
                }
            }
        }

        ArrayList<Double> yList = new ArrayList<>(50);
        for (AprilTagDetection detection : detectionList) {
            if (detection.metadata != null) {
                for (int i = 0; i < 50; i++) {
                    yList.set(i, detection.ftcPose.y);
                }
            }
        }


        ArrayList<Double> bearingList = new ArrayList<>(50);
        for (AprilTagDetection detection : detectionList) {
            if (detection.metadata != null) {
                for (int i = 0; i < 50; i++) {
                    bearingList.set(i, detection.ftcPose.bearing);
                }
            }
        }


        telemetry.addData("X VALUE RIGHT NOW THIS SECOND", tempX);
        telemetry.addData("X POSITION", calculateMean(xList));
        telemetry.addData("Y POSITION", calculateMean(yList));
        telemetry.addData("BEARING POSITION", calculateMean(bearingList));
        telemetry.update();



        return new Pose2d(new Vector2d(calculateMean(xList), calculateMean(yList)), calculateMean(bearingList));

    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }

    private double calculateMean(ArrayList<Double> list) {
        double sum = 0;
        for(double values : list) {
            sum += values;
        }
        return sum/list.size();
    }



}