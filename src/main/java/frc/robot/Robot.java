package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Vision;


public class Robot extends TimedRobot {

    @Override
    public void robotInit() {
        Drivetrain.init();
        Controllers.init();
        Vision.init();
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
        Drivetrain.loop();
        Vision.loop();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Left Encoder", Drivetrain.getLeftDistance());
        SmartDashboard.putNumber("Right Encoder", Drivetrain.getRightDistance());
        SmartDashboard.putNumber("Left Velocity", Drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", Drivetrain.getRightVelocity());
        SmartDashboard.putNumber("Gyro Heading", Drivetrain.getHeading());

        Drivetrain.checkResetGyro();
    }
}
