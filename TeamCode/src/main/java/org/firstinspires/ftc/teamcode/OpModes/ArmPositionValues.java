package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;




@TeleOp
public class ArmPositionValues extends LinearOpMode {

private Arm arm;
    private DcMotorEx armMotor;
private OpMode opMode;
private Telemetry telemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
//opmode needs waitFOrStart() and a way to exit if stopped
        if (isStopRequested()) return;

        while (!isStopRequested()) {
            arm = new Arm(this);
            opMode.telemetry.addData("Encoder value", armMotor.getCurrentPosition());
            opMode.telemetry.update();
        }


    }
}
