package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/**
 * DriveTrain subsystem
 * This class is a subsystem that contains the drive train of the robot.
 * We modified the RoadRunner MecanumDrive class to be not final so we can extend it.
 * We added a driveType enum to switch between robot centric and field centric driving.
 * We can then use the RR MecanumDrive class to drive the robot in the teleop.
 */
public class DriveTrain extends MecanumDrive {

    private LinearOpMode currentOpMode;

    public DriveTrain(HardwareMap hardwareMap, Pose2d pose, LinearOpMode opMode ) {
        super(hardwareMap, pose);
        currentOpMode = opMode;
    }

    public enum DriveType {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    }
    public DriveType driveType = DriveType.ROBOT_CENTRIC;

    public Vector2d gamepadInput = new Vector2d(0,0);
    public double gamepadInputTurn = 0;

    public void driveNormal(){
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        gamepadInput.x,
                        gamepadInput.y),
                gamepadInputTurn
        ));

        updatePoseEstimate();
    }

    public Vector2d rotateFieldCentric(double x, double y, double angle){
        double newX, newY;
        newX = x * Math.cos(angle) - y * Math.sin(angle);
        newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2d(newX, newY);
    }

    public String toStringVector2d(Vector2d vector){
        return String.format("(%.3f, %.3f)", vector.x, vector.y);
    }

    public String toStringPose2d(Pose2d pose){
        return String.format("(%.3f, %.3f, %.3f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.log()));
    }

    public void outputTelemetry(){

        Telemetry telemetry = currentOpMode.telemetry;
        telemetry.addData("Drive Type : ", driveType);
        telemetry.addLine("Localizer pose data:");
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        telemetry.addData("Parallel Left /LF Encoder", leftFront.getCurrentPosition());
        telemetry.addData("Parallel Right / RF Encoder", rightFront.getCurrentPosition());
        telemetry.addData("Perpendicular / RB Encoder", rightBack.getCurrentPosition());
        telemetry.addData("LB Encoder", leftBack.getCurrentPosition());
        telemetry.addLine("=============");
    }
}
