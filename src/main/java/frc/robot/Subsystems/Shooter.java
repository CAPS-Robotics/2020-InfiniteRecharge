package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

public class Shooter {
    private static WPI_TalonSRX motorA;
    private static WPI_TalonSRX motorB;

    public static void init() {
        motorA = new WPI_TalonSRX(RobotMap.SHOOTER_A);
        motorB = new WPI_TalonSRX(RobotMap.SHOOTER_B);
    }

    public static void loop() {

    }

    public static void setSpeed(double speed) {
        motorA.set(speed);
        motorB.set(speed);
    }
}