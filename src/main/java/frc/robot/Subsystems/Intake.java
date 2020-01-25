package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

public class Intake {
    private static WPI_TalonSRX motor;

    public static void init() {
        motor = new WPI_TalonSRX(RobotMap.INTAKE);
    }
    public static void loop() {

    }
    public static void setIntake() {
        motor.set(1);
    }
    public static void setOuttake() {
        motor.set(-1);
    }
    public static void stop() {
        motor.set(0);
    }
}
