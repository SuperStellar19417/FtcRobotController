package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

public class Utils {
    public static String toStringPose2d(Pose2d pose){
        return String.format("(%.3f, %.3f, %.3f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.log()));
    }

    public static void outputDriveTelemetry(Telemetry telemetry, GamepadController.DriveType driveType,
                                            MecanumDrive drive){

        telemetry.addData("Drive Type : ", driveType);
        telemetry.addLine("Localizer pose data:");
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));

        telemetry.addData("Parallel Left /LF Encoder", drive.leftFront.getCurrentPosition());
        telemetry.addData("Parallel Right / RF Encoder", drive.rightFront.getCurrentPosition());
        telemetry.addData("Perpendicular / RB Encoder", drive.rightBack.getCurrentPosition());
        telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
        telemetry.addLine("=============");
    }
}

