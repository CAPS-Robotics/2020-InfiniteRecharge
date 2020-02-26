package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.*;


public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        Drivetrain.init();
        Intake.init();
        Feeder.init();
        Shooter.init();
        Turret.init();
        Controllers.init();
        Vision.init();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        Drivetrain.loop();
        Intake.loop();
        Feeder.loop();
        Shooter.loop();
        Turret.loop();
        Vision.loop();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left Encoder", Drivetrain.getLeftDistance());
        SmartDashboard.putNumber("Right Encoder", Drivetrain.getRightDistance());
        SmartDashboard.putNumber("Left Velocity", Drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", Drivetrain.getRightVelocity());
        SmartDashboard.putNumber("Gyro Heading", Drivetrain.getHeading());
        SmartDashboard.putNumber("Wrist Angle", Intake.getWristAngle());
        SmartDashboard.putNumber("Turret Angle", Turret.getAngle());
        SmartDashboard.putBoolean("Field Orientated", Turret.isFieldOrientated());

        Drivetrain.checkResetGyro();
        Turret.checkResetAngle();
    }
}
