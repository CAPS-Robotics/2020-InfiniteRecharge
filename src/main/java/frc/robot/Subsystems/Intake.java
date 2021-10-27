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
        // mra: You may need to wait a few ms - Only do so if the retrieved value below
        //          is not what you think is correct.
        // mra: Also return the actual motor speed per the following comment
        // mra: You will need another output variable, ex. percentMotorSpeedRtv
        // mra: You'll need to program in a small delay so that you can read the value
        //         I think you will need Thread.sleep(milSec)
    }

    public static double getWristPower(){
        // mra: You're returning the value that intended to use for percent motor speed.
        // mra: Instead, return the actual wrist motor speed percent.
        // mra: To show actual percent, will need to multiply by 100
        //         percentMotorSpeedRtv = wrist.get()
         return wristPower;  // mra: Keep this value as it's the intended value
                            //         It's important to know.
    }


        //this is OUR code
        // Adjusted hard-coded values "wrist angles" to be actually relevant w/testing
    public static void wristUp ()
    {
        if (getWristAngle() < 25) {
            // mra: Separate the following out into multiple statements using local variables
            // mra: Use SmartDashboard to output the local values and then sleep long enough
            //         to see the values

            // mra: localvar1 = getWristAngle()
            // mra: Display localvar1 (variable name only intended as an example)
            // mra: localvar2 = 90 - localvar1
            // mra: Display localvar2
            // mra: localvar3 = localvar2 / 300
            // mra: Display localvar3

            // mra: Then rewrite to setWristPower(localvar3)
            setWristPower((90 - getWristAngle()) / 300);

            // mra: Now get the wrist motor "power" and display it here.

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

