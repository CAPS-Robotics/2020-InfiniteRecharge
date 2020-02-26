package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.RobotMap;

public class Turret {
    private static double TURRET_OFFSET = 0;
    private static final double MIN_SPEED = 0.1;
    private static final double TOLERANCE = 5;
    private static WPI_TalonSRX turretMotor;
    private static PIDController turretController;
    private static final double TURRET_P = 1/540d;
    private static final double TURRET_I = 0;
    private static final double TURRET_D = 0;
    private static boolean fieldOrientated;

    public static void init() {
        turretMotor = new WPI_TalonSRX(RobotMap.TURRET);
        turretMotor.setInverted(true);
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        turretController = new PIDController(TURRET_P, TURRET_I, TURRET_D, 0.002);
        turretController.setTolerance(5);
        turretController.setIntegratorRange(-0.05, 0.05);
        fieldOrientated = false;

        resetAngle();
    }

    public static void loop() {
        if(fieldOrientated) turretController.setSetpoint(-Drivetrain.getHeading());
        else if(Controllers.getPOVUp(false)) turretController.setSetpoint(0);
        else if(Controllers.getPOVRight(false)) {
            turretController.setSetpoint(90);
            turretController.reset();
        }
        else if(Controllers.getPOVLeft(false)) turretController.setSetpoint(-90);
        else if(Controllers.getPOVDown(false)) {
            if(getAngle() > 0) turretController.setSetpoint(180);
            else turretController.setSetpoint(-180);
        }

        /*if(Controllers.getLeftXAxis(false) < -0.05 || Controllers.getLeftXAxis(false) > 0.05) {
            setSpeed(Controllers.getLeftXAxis(false));
            turretController.setSetpoint(getAngle());
            turretController.calculate(getAngle());
        } else if(!atAngle()) {
            if(turretController.calculate(getAngle()) > -MIN_SPEED && turretController.calculate(getAngle()) < MIN_SPEED) {
                setSpeed(MIN_SPEED * (turretController.calculate(getAngle()) > 0 ? 1 : -1));
            } else {
                setSpeed(turretController.calculate(getAngle()));
            }
        }

        if(Controllers.getLeftJoyButton(false)) {
            fieldOrientated = !fieldOrientated;
            if(fieldOrientated) turretController.setP(1/270d);
            else turretController.setP(1/420d);
        }*/
        SmartDashboard.putNumber("Turret Target", turretController.getSetpoint());
        SmartDashboard.putBoolean("Turret at setpoint", atAngle());
        SmartDashboard.putNumber("Turret Current", turretMotor.getOutputCurrent());
        /*if(!atAngle()) {
            double targetPower = turretController.calculate(getAngle());
            if(targetPower > -MIN_SPEED && targetPower < MIN_SPEED) {
                setSpeed(MIN_SPEED * (targetPower > 0 ? 1 : -1));
            } else {
                setSpeed(targetPower);
            }
        } else {
            setSpeed(0);
        }*/
        if(atAngle()) setSpeed(0);
        else setSpeed(turretController.calculate(getAngle()));
        /*if(Math.abs(turretController.getSetpoint() - getAngle()) > 5) {
            if (turretController.getSetpoint() > getAngle()) setSpeed(0.1);
            else setSpeed(-0.1);
        } else setSpeed(0);*/

    }

    public static void setSpeed(double speed) {
        turretMotor.set(speed);
        SmartDashboard.putNumber("Turret Target Power", speed);
    }
    public static double getAngle() {
        return -((turretMotor.getSensorCollection().getPulseWidthPosition() / 5.5 * 360d / 4096d) - TURRET_OFFSET);
    }
    public static boolean atAngle() {
        return Math.abs(turretController.getSetpoint() - getAngle()) < TOLERANCE;
    }
    public static void checkResetAngle() {
        if(Controllers.getLeftStartButton(false)) resetAngle();
    }
    public static void resetAngle() {
        TURRET_OFFSET = turretMotor.getSensorCollection().getPulseWidthPosition() / 5.5 * 360d / 4096d;
        turretController.setSetpoint(0);
        fieldOrientated = false;
    }
    public static boolean isFieldOrientated() {
        return fieldOrientated;
    }
    public static void stop() {
        turretMotor.set(0);
    }
}
