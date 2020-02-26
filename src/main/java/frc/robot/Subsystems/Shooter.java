package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.RobotMap;

public class Shooter {
    private static CANSparkMax motorA;
    private static CANSparkMax motorB;
    private static double shooterPower;

    public static void init() {
        motorA = new CANSparkMax(RobotMap.SHOOTER_A, CANSparkMaxLowLevel.MotorType.kBrushless);
        motorB = new CANSparkMax(RobotMap.SHOOTER_B, CANSparkMaxLowLevel.MotorType.kBrushless);

        motorA.setInverted(false);

        shooterPower = 0;
    }

    public static void loop() {
        if(Controllers.getXButton(false)) shooterPower = 0;
        else if(Controllers.getYButton(false)) shooterPower += 0.05;
        else if(Controllers.getBButton(false)) shooterPower = 1;
        else if(Controllers.getAButton(false)) shooterPower -= 0.05;

        if(shooterPower < 0) shooterPower = 0;
        else if(shooterPower > 1) shooterPower = 1;

        setSpeed(shooterPower);

        SmartDashboard.putNumber("ShooterA Current", motorA.getOutputCurrent());
        SmartDashboard.putNumber("ShooterB Current", motorB.getOutputCurrent());
        SmartDashboard.putNumber("RPM", (motorA.getEncoder().getVelocity() * 2));
    }

    public static void setSpeed(double speed) {
        motorA.set(speed);
        motorB.set(speed);
    }
}