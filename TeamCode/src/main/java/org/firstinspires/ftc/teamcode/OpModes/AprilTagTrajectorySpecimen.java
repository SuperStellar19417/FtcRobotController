package org.firstinspires.ftc.teamcode.OpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.util.Size;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ArmMoveToHighBasket;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ArmMoveToHighBasketAgain;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ArmMoveToRestingPosition;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ArmMoveToRestingPositionAgain;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ClawClose;
import org.firstinspires.ftc.teamcode.OpModes.Actions.ClawOpen;
import org.firstinspires.ftc.teamcode.OpModes.Actions.MoveArmHighRung;
import org.firstinspires.ftc.teamcode.OpModes.Actions.MoveArmHighRungAgain;
import org.firstinspires.ftc.teamcode.OpModes.Actions.MoveArmMidway;
import org.firstinspires.ftc.teamcode.OpModes.Actions.MoveArmSlightlyUp;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesExtendToHighBasket;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesExtendToHighBasketAgain;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesExtendToSpecimenIntake;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesRetractFromSpecimenIntake;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesRetractToMin;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesRetractToMinAgain;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesSlightlyExtend;
import org.firstinspires.ftc.teamcode.OpModes.Actions.SlidesSlightlyRetract;
import org.firstinspires.ftc.teamcode.OpModes.Actions.WristToIntakePosition;
import org.firstinspires.ftc.teamcode.OpModes.Actions.WristToUpPosition;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Blue Specimen Autonomous", group = "01-Test")
public class AprilTagTrajectorySpecimen extends LinearOpMode {

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            2, -13, 9, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    // Locations of the tags on the field
    private Map<Integer, Pose2d> tagLocations = new HashMap<>();

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE_FROM_TAG = 12.0; //  this is how close the camera should get to the target (inches) (forward/back)
    final double MOVE_DISTANCE_LR = -14.0; //  How much left or right to move -ve is right, +ve is left from robot perspective
    final double MOVE_DISTANCE_FORWARD = 4.0; //  How much forward to move -ve is back, +ve is forward from robot perspective
    final double SAMPLE_PICKUP_DS_THRESHOLD = 3; //  How close to wall to get using distance sensor in inches
    final double DROP_OFF_DISTANCE_FROM_TAG = 24.0; //  this is how much backwards to move from the tag to drop off the specimen

    private static final int DESIRED_TAG_ID = 11;

    // Declare subsystems here
    private MecanumDrive driveTrain;
    private Arm arm;
    private IntakeSlide slides;
    private Claw claw;

    private ClawClose closeClaw;
    private ClawOpen openClaw;
    private SlidesExtendToHighBasket slidesHigh;
    private SlidesExtendToHighBasketAgain slidesHighAgain;
    private SlidesRetractToMin slidesLow;
    private SlidesRetractToMinAgain slidesLowAgain;
    private MoveArmSlightlyUp armSlightlyUp;
    private ArmMoveToRestingPosition armToIntake;
    private ArmMoveToRestingPositionAgain armToIntakeAgain;
    private SlidesSlightlyRetract retractSlides;
    private MoveArmMidway armMidway;


    private MoveArmHighRung armToHighRung;
    private MoveArmHighRungAgain armToHighRungAgain;
    private WristToIntakePosition wristIntake;
    private WristToUpPosition wristUp;
    private SlidesSlightlyExtend extendSlides;

    private SlidesExtendToSpecimenIntake slidesToSpecimenIntake;

    private SlidesRetractFromSpecimenIntake slidesRetractFromSpecimenIntake;

    private boolean gp1ButtonALast = false;

    public AprilTagTrajectorySpecimen() {
        // Initialize tag locations
        initTagLocation();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initAprilTag();

        setManualExposure(6, 500);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Get current field position based on AprilTag detection
        // Init drive train with the start position detected
        Pose2d startPose = new Pose2d(new Vector2d(12,-63), Math.toRadians(90));
        initSubsystems(startPose);

        // If Stop is pressed, exit OpMode
        if (isStopRequested()) return;

        telemetry.update();

        // Create a trajectory to first waypoint (somewhere we can see tag # 16)
        // Move forward 5 inches and turn 90 degrees CCW (heading 180 in RR coordinates)
        Action trajectoryAction = driveTrain.actionBuilder(startPose)
                .lineToY(startPose.position.y + 27)
                .build();


        Actions.runBlocking(new ParallelAction(trajectoryAction, armToHighRung, closeClaw));
        Actions.runBlocking(new SequentialAction(wristIntake));
        safeWaitSeconds(0.5);


        Pose2d tempPose = new Pose2d(new Vector2d(startPose.position.x, startPose.position.y + 20), Math.toRadians(90));
        Action moveBack = driveTrain.actionBuilder(tempPose)
                .lineToY(startPose.position.y + 4)
                .build();

        Actions.runBlocking(new SequentialAction(moveBack));
        safeWaitSeconds(0.25);
        Actions.runBlocking(new SequentialAction(openClaw, new ParallelAction(armToIntake, wristUp)));

        slides.runSlideMotorAllTheWayDown();
        safeWaitSeconds(0.75);

     //   Actions.runBlocking(new ParallelAction(moveBack, slidesLow, armToIntake, openClaw, wristUp));


        Action moveBackPosition = driveTrain.actionBuilder(tempPose)
                .lineToY(startPose.position.y + 11)
                .turnTo(Math.toRadians(0))
                .build();


        Actions.runBlocking(new SequentialAction(moveBackPosition));

     //   tempPose = new Pose2d(new Vector2d(12, -53), Math.toRadians(0));

        startPose = getFieldPosition(11);


        Action toShieldPreload = driveTrain.actionBuilder(startPose)
                .setTangent(Math.toRadians(180))
                .lineToX(startPose.position.x - 28)
                .build();

        Action precisionShield = driveTrain.actionBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .lineToY(startPose.position.y + 8)
                .setTangent(Math.toRadians(180))
                .lineToX(startPose.position.x - 30)
                .build();

        Action backFromShield = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x + 10, startPose.position.y + 5))
                .build();


        Actions.runBlocking(new ParallelAction(slidesToSpecimenIntake, toShieldPreload, openClaw));
        safeWaitSeconds(0.5);
        Actions.runBlocking(new SequentialAction(precisionShield));

        claw.intakeClawClose();
        safeWaitSeconds(0.2);
        Actions.runBlocking(new SequentialAction(armSlightlyUp));
        slides.runSlideMotorAllTheWayDown();
        safeWaitSeconds(0.75);
        Actions.runBlocking(new SequentialAction(backFromShield, armToHighRungAgain));
        Actions.runBlocking(new ParallelAction(wristUp));


        Pose2d newPose;
        newPose = new Pose2d(new Vector2d(startPose.position.x - 32, startPose.position.y + 7.5), Math.toRadians(0));

        Action pullBack = driveTrain.actionBuilder(newPose)
                .strafeTo(new Vector2d(newPose.position.x - 38, newPose.position.y + 11))
                .build();


        Action midPoseToSub = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y))
                .build();

        Actions.runBlocking(pullBack);


        startPose = getFieldPosition(11);

        safeWaitSeconds(0.5);

        Action toSubOne = driveTrain.actionBuilder(startPose)
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(startPose.position.x - 7, startPose.position.y - 18))
                .build();

        Actions.runBlocking(new SequentialAction(toSubOne));

        Actions.runBlocking(new SequentialAction(wristIntake));
        safeWaitSeconds(0.5);

        startPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(90));
        Action moveBackFromSub = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(0, -4))
                .build();

        Actions.runBlocking(new SequentialAction(armMidway));
        safeWaitSeconds(0.75);
        Actions.runBlocking(new SequentialAction(moveBackFromSub, openClaw, wristUp, armToIntakeAgain));



        startPose = new Pose2d(new Vector2d(startPose.position.x, startPose.position.y - 13), 90);

        Action midPoseSecond = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y - 5))
                .build();


    /*    Actions.runBlocking(new SequentialAction(midPoseSecond));

        claw.intakeClawOpen();
        safeWaitSeconds(0.3);
        claw.wristUp();

        Action pullMoreBack = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y - 15))
                .turn(Math.toRadians(-90))
                .build();

        Actions.runBlocking(new SequentialAction(pullMoreBack));


      /*  Action midPose = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y))
                .build(); */

        startPose = getFieldPosition(11);

        // ALL THIS HAPPENS AFTER THE PRELOADS
    /*    Action pushFieldSample = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x - 25, startPose.position.y))
                .strafeTo(new Vector2d(startPose.position.x - 26, startPose.position.y - 25))
                .strafeTo(new Vector2d(startPose.position.x - 26, startPose.position.y - 25))
                .strafeTo(new Vector2d(startPose.position.x - 26, startPose.position.y ))
              //  .strafeToLinearHeading(new Vector2d(-48,-48), Math.toRadians(225))
             //   .strafeToLinearHeading(new Vector2d(-46,-36), Math.toRadians(180))
               // .lineToY(startPose.position.y + 5)
             //   .turnTo(Math.toRadians(180))
           //     .lineToX(-48)
              //  .lineToYConstantHeading(startPose.position.y + 5)
                .build();

        Actions.runBlocking(new SequentialAction(pushFieldSample)); */







  //      startPose = getFieldPosition(16);
        telemetry.addData("DriveTrain Initialized with Pose x y h (inch/deg)",
                "%.2f %.2f %.2f", startPose.position.x, startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();






      /*  Pose2d moveToBucket = new Pose2d(new Vector2d(-54, -53), Math.toRadians(90));

        Action toBucket2 = driveTrain.actionBuilder(moveToBucket)
                .strafeTo(new Vector2d(moveToBucket.position.x - 10, moveToBucket.position.y - 4))
                .turnTo(Math.toRadians(225))
                .build();

        Action faceTag = driveTrain.actionBuilder(tempPose)
                .turnTo(Math.toRadians(180))
                .build();

        //assume u drop another here
        startPose = getFieldPosition(16);
        Action toPark = driveTrain.actionBuilder(startPose)
                .strafeTo(new Vector2d(startPose.position.x, startPose.position.y + 20))
                .strafeTo(new Vector2d(startPose.position.x + 20, startPose.position.y + 30))
                .build(); */


        // Run to waypoint


        // We should be able to see tag 16 now
//        Pose2d currentPose = getFieldPosition(16);
//        telemetry.addData("Second Pose x y h (inch/deg):",
//               "%.2f %.2f %.2f",
//                currentPose.position.x, currentPose.position.y,
//                Math.toDegrees(currentPose.heading.toDouble()));
//
//        telemetry.addLine("Press A for next step.");
//        telemetry.update();
//        while(!gp1GetButtonAPress() && opModeIsActive() && !isStopRequested()) {
//            sleep(20);
//        }

        // Create a trajectory to the next waypoint (tag 16)
        // Get to basket position with correct angle
//        trajectoryAction = driveTrain.actionBuilder(currentPose)
//                .splineTo(new Vector2d(-60, -60), Math.toRadians(-135))
//                .build();
        // Run to waypoint
     //   Actions.runBlocking(new SequentialAction(trajectoryAction));

        // Sit and loop till stop is pressed
        while (opModeIsActive() && !isStopRequested()) {

            // Update telemetry
            telemetry.update();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
    }

    private void initTagLocation() {
        tagLocations.put(11, new Pose2d(-72, 48, Math.toRadians(90)));
        tagLocations.put(12, new Pose2d(-0, 72, Math.toRadians(0)));
        tagLocations.put(13, new Pose2d(72, 48, Math.toRadians(270)));
        tagLocations.put(14, new Pose2d(72, -48, Math.toRadians(270)));
        tagLocations.put(15, new Pose2d(0, -72, Math.toRadians(180)));
        tagLocations.put(16, new Pose2d(-72, -40, Math.toRadians(90)));
    }

    private void initSubsystems(Pose2d startPose) {
        // Initialization code here

        driveTrain = new MecanumDrive(hardwareMap, startPose);
        arm = new Arm(this);
        slides = new IntakeSlide(this);
        claw = new Claw(this);
        telemetry.addData("DriveTrain Initialized with Pose x y h (inch/deg)",
                "%.2f %.2f %.2f", startPose.position.x, startPose.position.y,
                Math.toDegrees(startPose.heading.toDouble()));

        closeClaw = new ClawClose(claw, telemetry);
        openClaw = new ClawOpen(claw, telemetry);
        slidesHigh = new SlidesExtendToHighBasket(slides, telemetry);
        slidesLow = new SlidesRetractToMin(slides, telemetry);
        armToIntake = new ArmMoveToRestingPosition(arm, telemetry);
        wristIntake = new WristToIntakePosition(claw, telemetry);
        wristUp = new WristToUpPosition(claw, telemetry);
        slidesHighAgain = new SlidesExtendToHighBasketAgain(slides, telemetry);
        armToIntakeAgain = new ArmMoveToRestingPositionAgain(arm, telemetry);
        slidesToSpecimenIntake = new SlidesExtendToSpecimenIntake(slides, telemetry);
        slidesRetractFromSpecimenIntake = new SlidesRetractFromSpecimenIntake(slides, telemetry);
        armToHighRung = new MoveArmHighRung(arm, telemetry);
        armToHighRungAgain = new MoveArmHighRungAgain(arm, telemetry);
        extendSlides = new SlidesSlightlyExtend(slides, telemetry);
        retractSlides = new SlidesSlightlyRetract(slides, telemetry);
        armSlightlyUp = new MoveArmSlightlyUp(arm, telemetry);
        armMidway = new MoveArmMidway(arm, telemetry);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    /*
    Manually set the camera gain and exposure.
    This can only be called AFTER calling initAprilTag(), and only works for Webcams;
   */




    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private Pose2d getFieldPosition(int desiredTagID) {
        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

        List<AprilTagDetection> allDetections = new ArrayList<>();

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        AprilTagDetection desiredTag = null; // The AprilTag target we are looking for

        // get 10 reading so we can average them out
        while (allDetections.size() < 10 && opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            targetFound = false;
            desiredTag  = null;

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == desiredTagID) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is not in the library.
                    telemetry.addData("Skipping", "Tag ID %d is not in the library", detection.id);
                }
            }

            if (targetFound) {
                allDetections.add(desiredTag);
                if (allDetections.size() > 50) {
                    allDetections = allDetections.subList(0, 50);
                }
            }

            sleep(20); // Add a small delay to avoid overwhelming the CPU
        }
        visionPortal.setProcessorEnabled(aprilTag, false);

        // Calculate the average position of the tag
        if (allDetections.size() > 0) {
            double x = 0;
            double y = 0;
            double h = 0;

            for (AprilTagDetection detection : allDetections) {
                x += detection.robotPose.getPosition().x;
                y += detection.robotPose.getPosition().y;
                h += detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            }

            x /= allDetections.size();
            y /= allDetections.size();
            h /= allDetections.size();

            return new Pose2d(x, y, Math.toRadians(h + 90));
        }

        return null;
    }

    private boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && gamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = gamepad1.a;
        return isPressedButtonA;
    }

    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            //don't even worry about it
        }
    }

    private void moveRobot(double x, double y, double yaw) {
        // Implement your robot movement here
        driveTrain.setDrivePowers(new PoseVelocity2d(
                new Vector2d( x, y),
                yaw
        ));

        driveTrain.updatePoseEstimate();
    }

    private AprilTagDetection getAprilTagDetection(int tagId) {
        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((tagId < 0) || (detection.id == tagId)) {
                    // Yes, we want to use this tag.
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }

        return desiredTag;
    }

    private void moveJoyceForwardUsingAprilTag(int tagId, double offset) {
        // See https://ftc-docs.firstinspires.org/en/latest/apriltag/understanding_apriltag_detection_values/understanding-apriltag-detection-values.html#understanding-apriltag-detection-values
        // for how to interpret the values in the AprilTagDetection object.

        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        boolean done = false;               // Set to true when we are done with this task.

        visionPortal.setProcessorEnabled(aprilTag, true);
        while(opModeIsActive() && !isStopRequested() && !done)
        {
            AprilTagDetection desiredTag = getAprilTagDetection(tagId);
            if (desiredTag != null) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - offset);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Postion","Range Err %5.2f, Heading Err %5.2f, Yaw Err %5.2f ", rangeError, headingError, yawError);
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);

                // Check to see if we are close enough to the target.
                if (Math.abs(drive) <= 0.1 && Math.abs(strafe) <= 0.1 && Math.abs(turn) <= 0.1) {
                    done = true;
                }
            }

            telemetry.update();
            sleep(10);
        }

        moveRobot(0,0,0);
        visionPortal.setProcessorEnabled(aprilTag, false);
    }

    private void moveJoyceBackUsingAprilTag(int tagId, double offset) {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        double targetPosition   = 0;
        boolean done = false;               // Set to true when we are done with this task.

        visionPortal.setProcessorEnabled(aprilTag, true);
        AprilTagDetection desiredTag = getAprilTagDetection(tagId);
        if (desiredTag != null) {
            targetPosition = desiredTag.ftcPose.range + offset;
        }
        else {
            telemetry.addData("Error", "No April Tag detected");
            return;
        }

        telemetry.setAutoClear(true);
        while(opModeIsActive() && !isStopRequested() && !done)
        {
            desiredTag = getAprilTagDetection(tagId);
            if (desiredTag != null) {
                // Determine heading, range and Yaw (tag image rotation) error so we can use them to
                // control the robot automatically.

                // Since we are moving back, we need to reverse the range error
                double  rangeError      = (desiredTag.ftcPose.range - targetPosition);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                // drive = -drive; // Move backwards

                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Offset Range", "%5.2f %5.2f inches", offset, desiredTag.ftcPose.range);
                telemetry.addData("Postion","Range Err %5.2f, Heading Err %5.2f, Yaw Err %5.2f ", rangeError, headingError, yawError);
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);

                // Check to see if we are close enough to the target.
                if (Math.abs(drive) <= 0.1 && Math.abs(strafe) <= 0.1 && Math.abs(turn) <= 0.1) {
                    //telemetry.addData("Error", "Done");
                    done = true;
                }
            }
            telemetry.update();
            sleep(10);
        }

        visionPortal.setProcessorEnabled(aprilTag, false);
        moveRobot(0,0,0);
    }

    private void moveJoyceForwardUsingDistanceSensor(double offset) {
        boolean done  = false;    // Set to true when we are done with this task.
        double drivePower = 0.3;        // Desired forward power/speed (-1 to +1)

        while(opModeIsActive() && !isStopRequested() && !done)
        {
            double distance = claw.distanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Offset Distance", "%5.2f %5.2f inches", offset, distance);

            // Apply desired axes motions to the drivetrain.
            moveRobot(drivePower, 0, 0);

            // Check to see if we are close enough to the target.
            if ( distance <= offset) {
                done = true;
            }
            telemetry.update();
            sleep(10);
        }

        moveRobot(0, 0, 0);
    }

    private void moveJoyceBackUsingDistanceSensor(double offset) {
        boolean done = false;    // Set to true when we are done with this task.
        double drivePower = -0.3;        // Desired forward power/speed (-1 to +1)

        while(opModeIsActive() && !isStopRequested() && !done)
        {
            double distance = claw.distanceSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Offset Distance", "%5.2f %5.2f inches", offset, distance);

            // Apply desired axes motions to the drivetrain.
            moveRobot(drivePower, 0, 0);

            // Check to see if we are close enough to the target.
            if ( distance >= offset) {
                done = true;
            }
            telemetry.update();
            sleep(10);
        }

        moveRobot(0, 0, 0);
    }
}
