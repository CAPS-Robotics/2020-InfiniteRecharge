package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.MotionProfoling.Spline;
import frc.robot.MotionProfoling.VelocityProfile;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.*;

public class Drivetrain {
    private static final double GEARBOX_RATIO = 7;
    private static final double WHEEL_DIAMETER = 5;
    private static AHRS gyro;
    private static double gyroOffset;

    private static CANSparkMax leftMotorA;
    private static CANSparkMax leftMotorB;
    private static CANSparkMax rightMotorA;
    private static CANSparkMax rightMotorB;

    private static boolean start = true;

    public static void init() {
        leftMotorA = new CANSparkMax(RobotMap.LEFT_MOTOR_A, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftMotorB = new CANSparkMax(RobotMap.LEFT_MOTOR_B, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotorA = new CANSparkMax(RobotMap.RIGHT_MOTOR_A, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightMotorB = new CANSparkMax(RobotMap.RIGHT_MOTOR_B, CANSparkMaxLowLevel.MotorType.kBrushless);

        gyro = new AHRS(SPI.Port.kMXP);

        leftMotorA.restoreFactoryDefaults();
        leftMotorB.restoreFactoryDefaults();
        rightMotorA.restoreFactoryDefaults();
        rightMotorB.restoreFactoryDefaults();

        rightMotorA.setInverted(true);
        rightMotorB.setInverted(true);

        leftMotorA.getEncoder().setPosition(0);
        rightMotorA.getEncoder().setPosition(0);

        resetGyro();
    }

    public static void loop() {
        drive(Controllers.getLeftYAxis(true), Controllers.getRightYAxis(true));

        if(Controllers.getStartButton(true)) resetGyro();
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

    public static double getLeftDistance() {
        return leftMotorA.getEncoder().getPosition() / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI);
    }
    public static double getRightDistance() {
        return rightMotorA.getEncoder().getPosition() / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI);
    }
    public static double getLeftVelocity() {
        return leftMotorA.getEncoder().getVelocity() / 60 / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI) / 12;
    }
    public static double getRightVelocity() {
        return rightMotorA.getEncoder().getVelocity() / 60 / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI) / 12;
    }
    public static double getHeading() { return wrapAngle(gyro.getFusedHeading() - gyroOffset); }
    public static void resetGyro() { gyroOffset = gyro.getFusedHeading(); }
    private static double wrapAngle(double angle) {
        if(angle > 180) {
            return -(360 - angle);
        } else if (angle < -180) {
            return 360 - Math.abs(angle);
        }
        return angle;
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



