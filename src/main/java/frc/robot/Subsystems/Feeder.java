package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

public class Feeder {
    private static WPI_TalonSRX leftFeeder;
    private static WPI_TalonSRX rightFeeder;
    private static WPI_TalonSRX preRoller;

    public static void init() {
        leftFeeder = new WPI_TalonSRX(RobotMap.FEEDER_LEFT);
        rightFeeder = new WPI_TalonSRX(RobotMap.FEEDER_RIGHT);
        preRoller = new WPI_TalonSRX(RobotMap.PRE_ROLLER);
    }

    public static void setLeftSpeed(double speed) {
        leftFeeder.set(speed);
    }
    public static void setRightSpeed(double speed) {
        rightFeeder.set(speed);
    }
    public static void setPreRollerSpeed(double speed) {
        preRoller.set(speed);
    }
}
