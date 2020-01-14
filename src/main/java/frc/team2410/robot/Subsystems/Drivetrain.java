package frc.team2410.robot.Subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.team2410.robot.RobotMap;

public class Drivetrain {
    private static Spark leftMotorA;
    private static Spark leftMotorB;
    private static Spark rightMotorA;
    private static Spark rightMotorB;

    public static void init() {
        leftMotorA = new Spark(RobotMap.LEFT_MOTOR_A);
        leftMotorB = new Spark(RobotMap.LEFT_MOTOR_B);
        rightMotorA = new Spark(RobotMap.RIGHT_MOTOR_A);
        rightMotorB = new Spark(RobotMap.RIGHT_MOTOR_B);
    }

    public static void driveForward(double speed) {
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }

    public static void drive(double leftSpeed, double rightSpeed) {
        setLeftSpeed(leftSpeed);
        setRightSpeed(rightSpeed);
    }

    public static void stop() {
        setLeftSpeed(0);
        setRightSpeed(0);
    }

    private static void setLeftSpeed(double speed) {
        leftMotorA.set(speed);
        leftMotorB.set(speed);
    }
    private static void setRightSpeed(double speed) {
        rightMotorA.set(speed);
        rightMotorB.set(speed);
    }
}
