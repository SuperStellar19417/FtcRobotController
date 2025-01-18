package org.firstinspires.ftc.teamcode.OpModes.practiceOpmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SubSystems.GamepadController;

import java.util.List;

@TeleOp(name = "Practice Limelight", group = "Tests")
public class LimelightOpmode extends LinearOpMode {
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            if (isStopRequested())
                break;
            telemetry.addData("Recieve Result from limelight", "Red Result");
            if (result != null) {
                //telemetry.addData("Result is",result.toString());
                if (result.isValid()) {
                    Pose3D botpose= result.getBotpose();
                    telemetry.addData("Result is valid","Red Result 3");
                    telemetry.addData("tx:",result.getTx());
                    telemetry.addData("ty:",result.getTy());
                    telemetry.addData("ta:",result.getTa());
                    telemetry.addData("Botpose",botpose.toString());
                    telemetry.addData("Position",botpose.getPosition());

                    // Access fiducial results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    for (LLResultTypes.ColorResult cr : colorResults) {
                        telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    }
                }



            }
            else{
                telemetry.addData("Limelight","no target");
            }
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        limelight.stop();

    }
}
