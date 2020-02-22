package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotMap;

public class Intake {
    private static WPI_TalonSRX intakeMotor;
    private static WPI_TalonSRX wrist;

    public static void init() {
        intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE);
        wrist = new WPI_TalonSRX(RobotMap.WRIST);
    }
    public static void loop() {

    }
    public static void setIntake(double power) {
        intakeMotor.set(power);
    }
    public static void setWrist (double power) { wrist.set(power); }
}
