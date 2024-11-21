package org.firstinspires.ftc.teamcode.SubSystems;
public class HardwareConstant {
    public static String LeftFrontMotor = "leftFront"; // Expansion Hub Port 0
    public static String LeftBackMotor = "leftBack"; // Expansion Hub Port 1
    public static String RightBackMotor = "rightBack"; // Control Hub Port 1
    public static String RightFrontMotor = "rightFront"; // Control Hub Port 0

    // Odometry pods (these do not need to be defined in the config)
    public static String LeftOdoPod = "leftFront"; // Expansion Hub Port 0 encoder slot
    public static String RightOdoPod = "rightFront"; // Control Hub Port 0 encoder slot
    public static String LateralOdoPod = "rightBack"; // Control Hub Port 1 encoder slot

    public static String HeadlightLeft = "leftHeadlight";  //expansion hub servo port 0
    public static String HeadlightRight = "rightHeadlight"; //control hub servo port 5

    public static String SlideMotor = "linearSlideMotor";
    public static String ClawServo = "clawServo"; //control hub servo port 0
    public static String WristServo = "wristServo";
    public static String ClawColorSensor = "clawColorSensor";
    public static String ArmMotor = "armMotor";
    public static String LimitSwitch = "limitSwitch";
}
