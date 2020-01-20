package frc.team2410.robot.Subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.Spark;
import frc.team2410.robot.Controllers;
import frc.team2410.robot.RobotMap;

public class Drivetrain {
    private static Spark leftMotorA;
    private static Spark leftMotorB;
    private static Spark rightMotorA;
    private static Spark rightMotorB;

    private static CANCoder leftEncoder;
    private static CANCoder rightEncoder;

    public static void init() {
        leftMotorA = new Spark(RobotMap.LEFT_MOTOR_A);
        leftMotorB = new Spark(RobotMap.LEFT_MOTOR_B);
        rightMotorA = new Spark(RobotMap.RIGHT_MOTOR_A);
        rightMotorB = new Spark(RobotMap.RIGHT_MOTOR_B);

        leftEncoder = new CANCoder(RobotMap.LEFT_ENCODER);
        rightEncoder = new CANCoder(RobotMap.RIGHT_ENCODER);
    }

    public static void loop() {
        drive(Controllers.getLeftYAxis(true), Controllers.getRightYAxis(true));
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

    public static double getLeftDistance() { return leftEncoder.getPosition(); }
    public static double getRightDistance() { return rightEncoder.getPosition(); }
    public static double getLeftVelocity() { return leftEncoder.getVelocity(); }
    public static double getRightVelocity() { return rightEncoder.getVelocity(); }

    private static void setLeftSpeed(double speed) {
        leftMotorA.set(speed);
        leftMotorB.set(speed);
    }
    private static void setRightSpeed(double speed) {
        rightMotorA.set(speed);
        rightMotorB.set(speed);
    }
}



