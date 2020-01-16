package frc.team2410.robot.Subsystems;

import edu.wpi.first.wpilibj.Spark;
import frc.team2410.robot.RobotMap;

public class Shooter {
    private static Spark motorA;
    private static Spark motorB;

    public static void init() {
        motorA = new Spark(RobotMap.SHOOTER_A);
        motorB = new Spark(RobotMap.SHOOTER_B);
    }

    public static void loop() {

    }

    public static void setSpeed(double speed) {
        motorA.set(speed);
        motorB.set(speed);
    }
}