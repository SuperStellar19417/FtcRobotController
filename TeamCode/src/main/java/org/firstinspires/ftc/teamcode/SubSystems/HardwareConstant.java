package org.firstinspires.ftc.teamcode.SubSystems;
public class HardwareConstant {
    public static String LeftFrontMotor = "leftFront"; // Expansion Hub motor Port 0
    public static String LeftBackMotor = "leftBack"; // Expansion Hub motor Port 1
    public static String RightBackMotor = "rightBack"; // Control Hub motor Port 1
    public static String RightFrontMotor = "rightFront"; // Control Hub motor Port 0

    // Odometry pods (these do Not need to be defined in the config)
    public static String LeftOdoPod = "leftFront"; // Expansion Hub Port 0 encoder slot
    public static String RightOdoPod = "rightFront"; // Control Hub Port 0 encoder slot
    public static String LateralOdoPod = "rightBack"; // Control Hub Port 1 encoder slot

    public static String HeadlightLeft = "leftHeadlight";  //expansion hub servo port 0
    public static String HeadlightRight = "rightHeadlight"; //control hub servo port 5
    public static String HeadlightTop = "topHeadlight"; //


    public static String SlideMotor = "linearSlideMotor"; // control hub motor port 2
    public static String ClawServo = "clawServo"; //expansion hub servo port 0
    public static String WristServo = "wristServo"; // expansion hub servo port 3
    public static String ClawColorSensor = "clawColorSensor"; // control hub i2c bus 1
    public static String ArmMotor = "armMotor";  // expansion hub motor port 2
    public static String ClimberMotor = "climberMotor"; // expansion hub motor port 3
    public static String SlidesLimitSwitch = "slidesLimitSwitch"; // expansion hub digital port 3
    public static String ClimberLimitSwitch = "climberLimitSwitch"; // expansion hub digital port 7
    public static String DistanceSensor = "distanceSensor"; //control hub ic port 3
    public static String FlagServo = "flagServo"; // control hub servo port 0
}
