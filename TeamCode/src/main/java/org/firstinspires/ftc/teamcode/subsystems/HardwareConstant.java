package org.firstinspires.ftc.teamcode.subsystems;
public class HardwareConstant {

    // Drive Mortors
    public static String LeftFrontMotor = "frontLeftMotor"; // Expansion Hub Port 0
    public static String LeftBackMotor = "backLeftMotor"; // Expansion Hub Port 1
    public static String RightBackMotor = "backRightMotor"; // Control Hub Port 1
    public static String RightFrontMotor = "frontRightMotor"; // Control Hub Port 0

    // Odometry pods (these do not need to be defined in the config)
    public static String LeftOdometry = "leftOdometry"; // Expansion Hub Port 0 encoder slot
    public static String RightOdometry = "rightOdometry"; // Control Hub Port 0 encoder slot
    public static String LateralOdometry = "lateralOdometry"; // Control Hub Port 1 encoder slot

    public static String ServoIntakeArm = "servoIntakeArm";
    public static String RightIntakeServo = "rightIntakeServo";
    public static String LeftIntakeServo = "leftIntakeServo";
    public static String WristIntakeServo = "wristIntakeServo";
    public static String RightOuttakeServo = "rightOuttakeServo";
    public static String LeftOuttakeServo = "leftOuttakeServo";
    public static String WristOuttakeServo = "wristOuttakeServo";
}
