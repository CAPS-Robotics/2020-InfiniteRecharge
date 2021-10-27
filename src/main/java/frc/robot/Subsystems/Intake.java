package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Controllers;
import frc.robot.RobotMap;

public class Intake {
    private static WPI_TalonSRX intakeMotor;
    public static double wristPower;
    private static WPI_TalonSRX wrist;
    private static final double WRIST_ENCODER_OFFSET = -160;

    public static void init() {
        intakeMotor = new WPI_TalonSRX(RobotMap.INTAKE);
        wrist = new WPI_TalonSRX(RobotMap.WRIST);

        wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        intakeMotor.setInverted(true);
    }
    // TODO
    public static void loop() {
        if(Controllers.getLeftBumper(false)) setIntake(0.3);
        else setIntake(0);

        if(Controllers.deadzoneExceeded(Controllers.getRightYAxis(false), 0.3)) {
            if(Controllers.getRightYAxis(false) > 35) wristUp();
            else if (getWristAngle() > -45) wristDown();
        }
    }
    public static void setIntake(double power) {
        intakeMotor.set(power);
    }
    public static void setWristPower(double power) {
        wrist.set(power);
        wristPower = power;
    }

    public static double getWristPower(){
        return wristPower;
    }


        //this is OUR code
        // Adjusted hard-coded values "wrist angles" to be actually relevant w/testing
    public static void wristUp ()
    {
        if (getWristAngle() < 25) {
            setWristPower((90 - getWristAngle()) / 300);

     //   } else if (getWristAngle() < 60) {
     //       setWristPower((90 - getWristAngle()) / 200);
        } else if (getWristAngle() < -10) {
            setWristPower((90 - getWristAngle()) / 150);
        } else {
            setWristPower(0);
        }
    }
//  TODO Revisit angles
    public static void wristDown () {
        if (getWristAngle() > 20) {
            setWristPower(-getWristAngle() / 800);
        } else if ( 13 < getWristAngle()) {
            setWristPower(getWristAngle() / 400);
        } else if (-50 >= getWristAngle()){
            SmartDashboard.putNumber("Wrist Power", 0);
            setWristPower(0);
        }
    }
 //   turretController.setSetpoint(Drivetrain.getHeading());
 //   targetAngle = Drivetrain.getHeading();

  /*  public static void wristAngle () {
        if (getWristAngle() > 90) {
            setWrist(90);
        }
        if (getWristAngle() < 0) {
            setWrist(0);
        }
    }
*/
    public static double getWristAngle () {
        return -wrapAngle((wrist.getSensorCollection().getPulseWidthPosition() * 360d / 4096d + WRIST_ENCODER_OFFSET));
    }
    private static double wrapAngle (double angle){
        return angle;
    }
    public static double getWristCurrent () {
        return intakeMotor.getSupplyCurrent();
    }

    }

