package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Controllers;
import frc.robot.RobotMap;

public class Feeder {
    private static WPI_TalonSRX leftFeeder;
    private static WPI_TalonSRX rightFeeder;
    private static WPI_TalonSRX preRoller;
    private static boolean backwards;

    public static void init() {
        leftFeeder = new WPI_TalonSRX(RobotMap.FEEDER_LEFT);
        rightFeeder = new WPI_TalonSRX(RobotMap.FEEDER_RIGHT);
        preRoller = new WPI_TalonSRX(RobotMap.PRE_ROLLER);

        preRoller.setInverted(true);
        backwards = false;
    }
    public static void loop() {
        leftFeeder.set(Controllers.getLeftTrigger(false) * (backwards ? -1 : 1));
        rightFeeder.set(Controllers.getRightTrigger(false) * (backwards ? -1 : 1));
        if(Controllers.getRightBumper(false)) preRoller.set(1);
        else preRoller.set(0);
        if(Controllers.getRightStartButton(false)) backwards = !backwards;
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
