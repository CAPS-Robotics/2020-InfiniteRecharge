package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.MotionProfiling.VelocityProfile;
import frc.robot.RobotMap;
import com.kauailabs.navx.frc.*;

public class Drivetrain {
    private static final double GEARBOX_RATIO = 7;
    private static final double WHEEL_DIAMETER = 5;
    private static enum DRIVE_MODE {
        CONTROLLER_DRIVE,
        MOTION_DRIVE,
        GYRO_DRIVE
    }
    private static DRIVE_MODE driveMode;

    private static AHRS gyro;
    private static double gyroOffset;

    private static CANSparkMax leftMotorA;
    private static CANSparkMax leftMotorB;
    private static CANSparkMax rightMotorA;
    private static CANSparkMax rightMotorB;

    private static double leftOffset;
    private static double rightOffset;
    private static PIDController gyroController;

    private static Timer timer;

    public static final double GYRO_P = 1/360d;
    public static final double GYRO_I = 1.5;
    public static final double GYRO_D = 0;

    private static Timer test;
    public static void init() {
        driveMode = DRIVE_MODE.CONTROLLER_DRIVE;

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
        gyroController.setIntegratorRange(-0.05, 0.05);
        gyroController.setTolerance(3);

        resetGyro();

        timer = new Timer();
        test = new Timer();
        test.start();
    }

    public static void loop() {
        setControllerSpeed();
        checkTurnButtons();
        setTurnSpeed();
        checkAutoPathButtons();
        setAutoPathSpeed();
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
        return (((leftMotorA.getEncoder().getPosition() + leftMotorB.getEncoder().getPosition()) / 2) - leftOffset) / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI) / 12;
    }
    public static double getRightDistance() {
        return (((rightMotorA.getEncoder().getPosition() + rightMotorB.getEncoder().getPosition()) / 2) - rightOffset) / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI) / 12;
    }
    public static double getLeftVelocity() {
        return ((leftMotorA.getEncoder().getVelocity() + leftMotorB.getEncoder().getVelocity()) / 2) / 60 / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI) / 12;
    }
    public static double getRightVelocity() {
        return (leftMotorA.getEncoder().getVelocity() + leftMotorB.getEncoder().getVelocity()) / 2 / 60 / GEARBOX_RATIO * (WHEEL_DIAMETER * Math.PI) / 12;
    }
    public static void resetEncoders() {
        leftOffset = (leftMotorA.getEncoder().getPosition() + leftMotorB.getEncoder().getPosition()) / 2;
        rightOffset = (rightMotorA.getEncoder().getPosition() + rightMotorB.getEncoder().getPosition()) / 2;
    }

    public static void setControllerSpeed() {
        if(driveMode == DRIVE_MODE.CONTROLLER_DRIVE) {
            drive(Controllers.getDeadzone(Controllers.getLeftYAxis(true), 0.30), Controllers.getDeadzone(Controllers.getRightYAxis(true), 0.30));
        }
    }

    public static void checkAutoPathButtons() {
        if(Controllers.getLeftBumper(true) && driveMode != DRIVE_MODE.MOTION_DRIVE) {
            Timer processing = new Timer();
            processing.start();
            VelocityProfile.reset();
            resetEncoders();
            VelocityProfile.addWaypoint(0, 0, 0);
            VelocityProfile.addWaypoint(9, 9, 90);
            VelocityProfile.addWaypoint(15, 15, 45);
            VelocityProfile.generatePath(false);
            SmartDashboard.putNumber("Processing Time", processing.get());
            processing.stop();
            timer.reset();
            timer.start();
            driveMode = DRIVE_MODE.MOTION_DRIVE;
        }
    }
    public static void setAutoPathSpeed() {
        if(driveMode == DRIVE_MODE.MOTION_DRIVE) {
            VelocityProfile.calculateVelocities((getLeftDistance() + getRightDistance()) / 2 + 0.2);
            drive(VelocityProfile.getCurrentLeftVelocity() / VelocityProfile.MAX_VELOCITY + getVelocityFeedback(VelocityProfile.getCurrentLeftVelocity(), getLeftVelocity()) + getAngleFeedback(true), VelocityProfile.getCurrentRightVelocity() / VelocityProfile.MAX_VELOCITY + getVelocityFeedback(VelocityProfile.getCurrentRightVelocity(), getRightVelocity()) + getAngleFeedback(false));
            SmartDashboard.putNumber("left velocity", VelocityProfile.getCurrentLeftVelocity());
            if((getLeftDistance() + getRightDistance()) / 2 >= VelocityProfile.getPathDistance()) driveMode = DRIVE_MODE.CONTROLLER_DRIVE;
            SmartDashboard.putNumber("Gyro Error", getHeading() - VelocityProfile.getCurrentAngle());
            SmartDashboard.putNumber("Expected gyro", VelocityProfile.getCurrentAngle());
        }
    }
    private static double getVelocityFeedback(double pathVelocity, double currentVelocity) {
        //return 0.15 * ((pathVelocity - currentVelocity) / (VelocityProfile.MAX_VELOCITY));
        return 0;
    }
    private static double getAngleFeedback(boolean left) {
        return 0;
        /*if(getHeading() > 170 && VelocityProfile.getCurrentAngle() < -170 || getHeading() < -170 && VelocityProfile.getCurrentAngle() > 170) {
            double error = (180 - Math.abs(getHeading())) + (180 - Math.abs(VelocityProfile.getCurrentAngle()));
            return error / 180 * (getHeading() > VelocityProfile.getCurrentAngle() ? (left ? 1 : -1) : (left ? -1 : 1));
        }
        return 0.75 * ((getHeading() - (VelocityProfile.getCurrentAngle())) / 180 * (left ? -1 : 1));*/
    }


    public static double getHeading() { return wrapAngle(gyro.getFusedHeading() - gyroOffset); }
    public static void resetGyro() {
        gyroOffset = gyro.getFusedHeading();
        resetEncoders();
    }
    public static boolean atAngle() {
        return gyroController.atSetpoint();
    }

    public static double wrapAngle(double angle) {
        if(angle > 180) {
            return -(360 - angle);
        } else if (angle < -180) {
            return 360 - Math.abs(angle);
        }
        return angle;
    }

    private static void checkTurnButtons() {
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
    }
    public static void checkResetGyro() {
        if(Controllers.getStartButton(true)) resetGyro();
    }
    public static void setGyroHeading(double heading) {
        gyroController.reset();
        gyroController.setSetpoint(heading);
        driveMode = DRIVE_MODE.GYRO_DRIVE;
    }
    public static double getTargetHeading() { return gyroController.getSetpoint(); }
    public static void setTurnSpeed() {
        if (driveMode == DRIVE_MODE.GYRO_DRIVE) {
            drive(gyroController.calculate(getHeading()), -gyroController.calculate(getHeading()));
            if(gyroController.atSetpoint()) {
                driveMode = DRIVE_MODE.CONTROLLER_DRIVE;
            }
        }
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



