package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autonomous.Autonomous;
import frc.robot.Subsystems.*;


public class Robot extends TimedRobot {
    SendableChooser<Autonomous.AUTO_STATIONS> autoPicker;

    @Override
    public void robotInit() {
        Drivetrain.init();
        Intake.init();
        Feeder.init();
        Shooter.init();
        Turret.init();
        Controllers.init();
        Vision.init();
        Autonomous.init();

        autoPicker = new SendableChooser();
        autoPicker.addOption("Right Trench", Autonomous.AUTO_STATIONS.RIGHT_TRENCH);
        SmartDashboard.putData(autoPicker);
    }
    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left Encoder", Drivetrain.getLeftDistance());
        SmartDashboard.putNumber("Right Encoder", Drivetrain.getRightDistance());
        SmartDashboard.putNumber("Left Velocity", Drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", Drivetrain.getRightVelocity());
        SmartDashboard.putNumber("Gyro Heading", Drivetrain.getHeading());
        SmartDashboard.putNumber("Wrist Angle", Intake.getWristAngle());
        SmartDashboard.putNumber("Wrist Power", Intake.getWristPower());
        SmartDashboard.putNumber("Turret Angle", Turret.getAngle());
        SmartDashboard.putBoolean("Field Orientated", Turret.isFieldOrientated());
        SmartDashboard.putNumber("Wrist Current", Intake.getWristCurrent());
        SmartDashboard.putNumber("Wrist Current", Intake.getWristCurrent());
        SmartDashboard.putNumber("Left Y Axis 2", Controllers.getRightYAxis(false));


        SmartDashboard.putBoolean("Feeder Front Left", Feeder.getLeftFront());
        SmartDashboard.putBoolean("Feeder Back Left", Feeder.getLeftBack());
        SmartDashboard.putBoolean("Feeder Front Right", Feeder.getRightFront());
        SmartDashboard.putBoolean("Feeder Back Right", Feeder.getRightBack());
        SmartDashboard.putBoolean("Beam Break", Feeder.getPreRoller());
        SmartDashboard.putBoolean("Left Ball Back", Feeder.isLeftBack());
        SmartDashboard.putBoolean("Right Ball Back", Feeder.isRightBack());
        SmartDashboard.putNumber("Vision distance", Vision.getDistance());
        Drivetrain.checkResetGyro();
        Turret.checkResetAngle();
    }

    @Override
    public void autonomousInit() {
        Autonomous.setAuto(autoPicker.getSelected());
    }
    @Override
    public void autonomousPeriodic() {
        // mra: Entry from WPILib autonomous mode
        Drivetrain.loop();
        Intake.loop();
        Feeder.loop();
        Shooter.loop();
        Turret.loop();
        Vision.loop();
    }

    @Override
    public void teleopInit() {
        // mra: Set code here to run prior to teleopPeriodic initiation
    }
    @Override
    public void teleopPeriodic() {
        // mra: Entry from WPILib teleop mode
        Drivetrain.loop();
        Intake.loop();
        Feeder.loop();
        Shooter.loop();
        Turret.loop();
        Vision.loop();
    }
}
