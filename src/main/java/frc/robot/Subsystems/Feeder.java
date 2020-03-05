package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.RobotMap;

public class Feeder {
    private static double SIDE_ROLLER_SPEED = 0.25;
    private static double PRE_ROLLER_SPEED = 0.25;

    private static WPI_TalonSRX leftFeeder;
    private static WPI_TalonSRX rightFeeder;
    private static WPI_TalonSRX preRoller;

    private static AnalogInput leftFront;
    private static AnalogInput leftBack;
    private static AnalogInput rightFront;
    private static AnalogInput rightBack;

    private static DigitalInput beamBreak;

    private static boolean backwards;

    public static void init() {
        leftFeeder = new WPI_TalonSRX(RobotMap.FEEDER_LEFT);
        rightFeeder = new WPI_TalonSRX(RobotMap.FEEDER_RIGHT);
        preRoller = new WPI_TalonSRX(RobotMap.PRE_ROLLER);

        leftFront = new AnalogInput(RobotMap.FEEDER_LEFT_FRONT);
        leftBack = new AnalogInput(RobotMap.FEEDER_LEFT_BACK);
        rightFront = new AnalogInput(RobotMap.FEEDER_RIGHT_FRONT);
        rightBack = new AnalogInput(RobotMap.FEEDER_RIGHT_BACK);

        beamBreak = new DigitalInput(RobotMap.FEEDER_BEAM_BREAK);

        preRoller.setInverted(false);
        backwards = false;
    }
    public static void loop() {
        leftFeeder.set(Controllers.getLeftTrigger(false) * (backwards ? -1 : 1));
        rightFeeder.set(Controllers.getRightTrigger(false) * (backwards ? -1 : 1));
        if(Controllers.getRightBumper(false)) preRoller.set(PRE_ROLLER_SPEED);
        else preRoller.set(0);
        if(Controllers.getRightStartButton(false)) backwards = !backwards;

        SmartDashboard.putNumber("Left Front", leftFront.getVoltage());
        SmartDashboard.putNumber("Right Front", rightFront.getVoltage());
    }

    private static void indexBalls() {
        if(!getPreRoller()) {
            setPreRoller(PRE_ROLLER_SPEED);
            setLeftSpeed(SIDE_ROLLER_SPEED);
            setRightSpeed(SIDE_ROLLER_SPEED);
        } else {
            setPreRoller(0);
        }
    }

    public static void setLeftSpeed(double speed) {
        leftFeeder.set(speed);
    }
    public static void setRightSpeed(double speed) {
        rightFeeder.set(speed);
    }
    public static void setPreRoller(double speed) {
        preRoller.set(speed);
    }

    public static boolean getLeftFront() { return leftFront.getVoltage() > 1; }
    public static boolean getLeftBack() { return leftBack.getVoltage() > 1; }
    public static boolean getRightFront() { return rightFront.getVoltage() > 1; }
    public static boolean getRightBack() { return rightBack.getVoltage() > 1; }
    public static boolean getPreRoller() { return !beamBreak.get(); }
}
