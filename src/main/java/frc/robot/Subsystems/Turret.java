package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

public class Turret {
    private static WPI_TalonSRX motorA;

    public static void init() {
        motorA = new WPI_TalonSRX(RobotMap.TURRET_A);
    }

    public static void loop() {

    }

    public static void setSpeed(double speed) {
        motorA.set(speed);
    }

    public static void stop() {motorA.set(0);}
}
