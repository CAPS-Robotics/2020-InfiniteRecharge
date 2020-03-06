package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.RobotMap;

public class Shooter {
    private static final double MAX_RPM = 10000;
    private static final double RPM_INCREMENT = 100;
    private static final double SHOOTER_P = 0.00055;
    private static final double SHOOTER_I = 0.00195;
    private static final double SHOOTER_D = 0.000046;

    private static CANSparkMax motorA;
    private static CANSparkMax motorB;

    private static PIDController shooterController;
    private static double targetRPM;

    public static void init() {
        motorA = new CANSparkMax(RobotMap.SHOOTER_A, CANSparkMaxLowLevel.MotorType.kBrushless);
        motorB = new CANSparkMax(RobotMap.SHOOTER_B, CANSparkMaxLowLevel.MotorType.kBrushless);

        motorA.setInverted(false);

        shooterController = new PIDController(SHOOTER_P, SHOOTER_I, SHOOTER_D, 0.02);
        targetRPM = 0;
    }

    public static void loop() {
        if(Controllers.getXButton(false)) targetRPM = 0;
        else if(Controllers.getYButton(false)) targetRPM += RPM_INCREMENT;
        else if(Controllers.getBButton(false)) targetRPM = MAX_RPM;
        else if(Controllers.getAButton(false)) targetRPM -= RPM_INCREMENT;

        if(targetRPM < 0) targetRPM = 0;
        else if(targetRPM > MAX_RPM) targetRPM = MAX_RPM;

        setSpeed(calculatePower());

        SmartDashboard.putNumber("ShooterA Current", motorA.getOutputCurrent());
        SmartDashboard.putNumber("ShooterB Current", motorB.getOutputCurrent());
        SmartDashboard.putNumber("RPM", getRPM());
    }

    public static void setRPM(double rpm) {
        targetRPM = rpm;
    }
    public static void setSpeed(double speed) {
        motorA.set(speed);
        motorB.set(speed);
    }
    public static double getRPM() {
        return (motorA.getEncoder().getVelocity() + motorB.getEncoder().getVelocity()) / 2 * 2;
    }
    private static double calculatePower() {
        SmartDashboard.putNumber("Shooter Error", targetRPM - getRPM());
        shooterController.setSetpoint(targetRPM);
        if(targetRPM == 0) return 0;
        return targetRPM / MAX_RPM + shooterController.calculate(getRPM());
    }
}