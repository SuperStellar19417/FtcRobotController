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

    public static String ServoIntakeArm = "servoIntakeArm";
    public static String RightIntakeServo = "rightIntakeServo";
    public static String LeftIntakeServo = "leftIntakeServo";
    public static String WristIntakeServo = "wristIntakeServo";
    public static String RightOuttakeServo = "rightOuttakeServo";
    public static String LeftOuttakeServo = "leftOuttakeServo";
    public static String WristOuttakeServo = "wristOuttakeServo";
    public static String ClawColorSensor = "clawColorSensor";
}
