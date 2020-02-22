package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

public class Turret {
    private static WPI_TalonSRX turretMotor;

    public static void init() {
        turretMotor = new WPI_TalonSRX(RobotMap.TURRET);
    }

    public static void loop() {

    }

    public static void setSpeed(double speed) {
        turretMotor.set(speed);
    }

    public static void stop() {
        turretMotor.set(0);}
}
