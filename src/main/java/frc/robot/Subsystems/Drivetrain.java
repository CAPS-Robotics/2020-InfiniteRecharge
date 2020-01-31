package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
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

    private static PIDController gyroController;
    private static boolean gyroTurn;

    public static final double GYRO_P = 1/360d;
    public static final double GYRO_I = 0; //.00125
    public static final double GYRO_D = 0;

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

        gyroController = new PIDController(GYRO_P, GYRO_I, GYRO_D, 0.002);
        gyroController.enableContinuousInput(-180, 180);
        gyroController.setTolerance(3);
        gyroTurn = false;
        resetGyro();
        setGyroHeading(getHeading());
    }

    public static void loop() {
        //drive(Controllers.getLeftYAxis(true), Controllers.getRightYAxis(true));
        //if (!gyroController.atSetpoint() && gyroTurn) {
            //setTurnSpeed();
        //} else if (gyroController.atSetpoint()) {
            //gyroTurn = false;
        //}
        if (!gyroController.atSetpoint() && gyroTurn) {
            setTurnSpeed();
            gyroTurn = false;
            //stop();
        }
        if(Controllers.getStartButton(true)) resetGyro();
        if(Controllers.getYButton(true)) {
            setGyroHeading(0);
        }
        if(Controllers.getBButton(true)) {
            setGyroHeading(90);
        }
        if(Controllers.getAButton(true)) {
            setGyroHeading(180);
        }
        if(Controllers.getXButton(true)) {
            setGyroHeading(-90);
        }
        SmartDashboard.putNumber("PID Output", gyroController.calculate(getHeading()));
        SmartDashboard.putNumber("PID Error", gyroController.getPositionError());
        SmartDashboard.putNumber("PID Speed", gyroController.getVelocityError());
        SmartDashboard.putBoolean("Gyro turn", gyroTurn);
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
    public static void resetGyro() {
        gyroOffset = gyro.getFusedHeading();
        setGyroHeading(getHeading());
    }
    private static double wrapAngle(double angle) {
        if(angle > 180) {
            return -(360 - angle);
        } else if (angle < -180) {
            return 360 - Math.abs(angle);
        }
        return angle;
    }

    public static void setGyroHeading(double heading) {
        gyroController.setSetpoint(heading);
        gyroTurn = true;
    }
    public static double getTargetHeading() { return gyroController.getSetpoint(); }
    public static void setTurnSpeed() {
        drive(gyroController.calculate(getHeading()), -gyroController.calculate(getHeading()));
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



