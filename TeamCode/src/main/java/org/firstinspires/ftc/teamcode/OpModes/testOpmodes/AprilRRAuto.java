package org.firstinspires.ftc.teamcode.OpModes.testOpmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@TeleOp(name="April RR Auto", group = "Concept")
public class AprilRRAuto extends LinearOpMode {
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
            0, 4, 9, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private Map<Integer, Pose2D> tagLocations = new HashMap<>();

    public AprilRRAuto() {
        tagLocations.put(11, new Pose2D(DistanceUnit.INCH,-72, 48, AngleUnit.DEGREES, 90));
        tagLocations.put(12, new Pose2D(DistanceUnit.INCH,-0, 72, AngleUnit.DEGREES, 0));
        tagLocations.put(13, new Pose2D(DistanceUnit.INCH,72, 48, AngleUnit.DEGREES, 270));
        tagLocations.put(14, new Pose2D(DistanceUnit.INCH,72, -48, AngleUnit.DEGREES, 270));
        tagLocations.put(15, new Pose2D(DistanceUnit.INCH,0, -72, AngleUnit.DEGREES, 180));
        tagLocations.put(16, new Pose2D(DistanceUnit.INCH,-72, -40, AngleUnit.DEGREES, 90));
    }

    @Override
    public void runOpMode() {
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            telemetryAprilTag();

            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
            if (gamepad1.dpad_down) {
                visionPortal.stopStreaming();
            } else if (gamepad1.dpad_up) {
                visionPortal.resumeStreaming();
            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();
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
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getPosition().z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                Pose2D fieldPos = getFieldPosition(detection.robotPose, detection.id);
                 } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

    }   // end method telemetryAprilTag()

    private Pose2D getFieldPosition(Pose3D detectionPose, int tagID) {

        // Get Tag pose
        Pose2D tagLocation = tagLocations.get(tagID);
        if (tagLocation == null) {
            return null;
        }
        double tagLocationX = tagLocation.getX(DistanceUnit.INCH);
        double tagLocationY = tagLocation.getY(DistanceUnit.INCH);
        double tagLocationH = tagLocation.getHeading(AngleUnit.DEGREES);

        telemetry.addLine(String.format("Tag Position (x,y,h) %6.1f %6.1f %6.1f",
                tagLocationX,
                tagLocationY,
                tagLocationH));


        double detectionX = detectionPose.getPosition().x;
        double detectionY= detectionPose.getPosition().y;
        double angle = detectionPose.getOrientation().getYaw(AngleUnit.DEGREES);

        double fieldX = 0.0;
        double fieldY = 0.0;
        double fieldH = 0.0;

        Pose2D fieldPos;

        switch (tagID) {
            case 11:
                fieldX = tagLocationX + detectionY;
                fieldY = tagLocationY + detectionX;
                fieldH = angle + 90;
                break;
            case 12:
                fieldX = tagLocationX + detectionX;
                fieldY = tagLocationY + detectionY;
                fieldH = angle + 90;
                break;
            case 13:
                fieldX = tagLocationX + detectionX;
                fieldY = tagLocationY + detectionY;
                angle = angle + 90;
                break;
            case 14:
                fieldX = tagLocationX + detectionX;
                fieldY = tagLocationY + detectionY;
                fieldH = angle + 90;
                break;
            case 15:
                fieldX = tagLocationX + detectionX;
                fieldY = tagLocationY + detectionY;
                fieldH = angle + 90;
                break;
            case 16:
                fieldX = tagLocationX + detectionX;
                fieldY = tagLocationY + detectionY;
                fieldH = angle + 90;
                break;
        }
         fieldPos = new Pose2D(
                DistanceUnit.INCH,
                fieldX,
                fieldY,
                AngleUnit.DEGREES, fieldH);

        telemetry.addLine(String.format("Field Position (x,y,h) %6.1f %6.1f %6.1f",
                fieldPos.getX(DistanceUnit.INCH), fieldPos.getY(DistanceUnit.INCH), fieldPos.getHeading(AngleUnit.DEGREES)));

        return fieldPos;
    }
}
