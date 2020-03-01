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
    private static final double WRIST_ENCODER_OFFSET = -518.5546875;

    private static PIDController wristController;

    public static final double WRIST_P = 1/180d;
    public static final double WRIST_I = 0;
    public static final double WRIST_D = 0;

    public static void init() {
        intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE);
        wrist = new WPI_TalonSRX(RobotMap.WRIST);

        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        intakeMotor.setInverted(true);

        wristController = new PIDController(WRIST_P, WRIST_I, WRIST_D, 0.002);
        wristController.setIntegratorRange(-0.05, 0.05);
        wristController.disableContinuousInput();
        wristController.setTolerance(5);
    }
    public static void loop() {
        if(Controllers.getLeftBumper(false)) setIntake(0.5);
        else setIntake(0);

        if(Controllers.getPOVUp(false)) wristUp();
        else if(Controllers.getPOVDown(false)) wristDown();
        else setWrist(Controllers.getRightYAxis(false));

        SmartDashboard.putNumber("Wrist Power", -wristController.calculate(getWristAngle()));
        SmartDashboard.putNumber("Wrist Target", wristController.getSetpoint());
    }
    public static void setIntake(double power) {
        intakeMotor.set(power);
    }
    public static void setWrist (double power) { wrist.set(power); }
    public static void wristUp() {
        wristController.setSetpoint(90);
        //setWrist(-wristController.calculate(getWristAngle()));
    }
    public static void wristDown() {
        wristController.setSetpoint(0);
        //setWrist(-wristController.calculate(getWristAngle()));
    }

    public static double getWristAngle() {
        return wrapAngle(-(wrist.getSensorCollection().getPulseWidthPosition() * 360d / 4096d + WRIST_ENCODER_OFFSET));
    }
    private static double wrapAngle(double angle) {
        return 360 - angle;
    }
}
