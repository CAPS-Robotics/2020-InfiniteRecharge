package frc.robot.Subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.RobotMap;

public class Intake {
    private static WPI_TalonSRX intakeMotor;
    private static WPI_TalonSRX wrist;
    private static final double WRIST_ENCODER_OFFSET = -160;

    public static void init() {
        intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE);
        wrist = new WPI_TalonSRX(RobotMap.WRIST);

        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        intakeMotor.setInverted(true);
    }
    public static void loop() {
        if(Controllers.getLeftBumper(false)) setIntake(0.3);
        else setIntake(0);

        if(Controllers.deadzoneExceeded(Controllers.getRightYAxis(false), 0.3)) {
            if(Controllers.getRightYAxis(false) > 0) wristUp();
            else wristDown();
        }
    }
    public static void setIntake(double power) {
        intakeMotor.set(power);
    }
    public static void setWrist (double power) { wrist.set(power); }
    public static void wristUp() {
        if(getWristAngle() < 15) {
            setWrist((90 - getWristAngle()) / 300);
        } else if(getWristAngle() < 60) {
            setWrist((90 - getWristAngle()) / 200);
        }
        else if(getWristAngle() < 90) {
            setWrist((90 - getWristAngle()) / 150);
        } else {
            setWrist(0);
        }
    }

    public static void wristDown() {
        if(getWristAngle() > 75) {
            setWrist(-getWristAngle() / 800);
        } else if (getWristAngle() > 10) {
            setWrist(getWristAngle() / 500);
        } else {
            SmartDashboard.putNumber("Wrist Power", 0);
            setWrist(0);
        }
    }

    public static double getWristAngle() {
        return -wrapAngle((wrist.getSensorCollection().getPulseWidthPosition() * 360d / 4096d + WRIST_ENCODER_OFFSET));
    }
    private static double wrapAngle(double angle) {
        return angle;
    }
    public static double getWristCurrent() {
        return intakeMotor.getSupplyCurrent();
    }
}
