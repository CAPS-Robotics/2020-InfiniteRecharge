package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.RobotMap;

// import javax.naming.ldap.Control;

public class Turret {
    private static double TURRET_OFFSET = 0;
    private static final double TOLERANCE = 5;
    private static WPI_TalonSRX turretMotor;
    private static PIDController turretController;
    private static final double TURRET_P = 0.0060;
    private static final double TURRET_I = 0;
    private static final double TURRET_D = 0;
    private static double targetAngle;
    private static boolean fieldOrientated;

    public static void init() {
        turretMotor = new WPI_TalonSRX(RobotMap.TURRET);
        turretMotor.setInverted(true);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        turretController = new PIDController(TURRET_P, TURRET_I, TURRET_D, 0.02);
        turretController.setTolerance(5);
        fieldOrientated = false;

        resetAngle();
    }

    public static void loop() {
        if(Controllers.deadzoneExceeded(Controllers.getLeftXAxis(false), 0.05)) {
            setSpeed(Controllers.getLeftXAxis(false) / 5);
            turretController.setSetpoint(getAngle());
            targetAngle = getAngle();
        }  else if(Controllers.getLeftJoyButton(false)) {
            turretController.setSetpoint(Drivetrain.getHeading() + Vision.getAngle());
            targetAngle = Drivetrain.getHeading() + Vision.getAngle();
            setSpeed(setTurnSpeed());
        } else {
            SmartDashboard.putNumber("Turret Power", turretController.calculate(getAngle()));
            setSpeed(setTurnSpeed());
        }
        if(Controllers.getPOVUp(false)) targetAngle = 0;
        else if(Controllers.getPOVRight(false))  targetAngle = 90;
        else if(Controllers.getPOVLeft(false)) targetAngle = -90;
        else if(Controllers.getPOVDown(false)) {
            if(getAngle() > 0) targetAngle = 180;
            else targetAngle = -180;
        }
        SmartDashboard.putNumber("Turret Error", turretController.getSetpoint() - getAngle());
    }

    private static double setTurnSpeed() {
        /*if(Math.abs(targetAngle - getAngle()) < TOLERANCE) return 0;
        double power = ((targetAngle - getAngle()) / 600d);
        if(power <= MIN_SPEED && power >= -MIN_SPEED) return power > 0 ? MIN_SPEED : -MIN_SPEED;
        return power;*/
        turretController.setSetpoint(targetAngle);
        return turretController.calculate(getAngle());
    }
    public static void setSpeed(double speed) {
        turretMotor.set(speed);
        SmartDashboard.putNumber("Turret Target Power", speed);
    }
    public static double getAngle() {
        return -((turretMotor.getSensorCollection().getQuadraturePosition() / 5.5 * 360d / 4096d) - TURRET_OFFSET);
    }
    public static boolean atAngle() {
        return Math.abs(turretController.getSetpoint() - getAngle()) < TOLERANCE;
    }
    public static void checkResetAngle() {
        if(Controllers.getLeftStartButton(false)) resetAngle();
    }
    public static void resetAngle() {
        TURRET_OFFSET = turretMotor.getSensorCollection().getQuadraturePosition() / 5.5 * 360d / 4096d;
        turretController.setSetpoint(0);
        targetAngle = 0;
        fieldOrientated = false;
    }
    public static boolean isFieldOrientated() {
        return fieldOrientated;
    }
    public static void stop() {
        turretMotor.set(0);
    }
}
