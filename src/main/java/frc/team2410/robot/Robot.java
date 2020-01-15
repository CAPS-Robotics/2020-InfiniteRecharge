package frc.team2410.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team2410.robot.Subsystems.Drivetrain;

public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        Drivetrain.init();
        Controllers.init();
    }

    @Override
    public void teleopPeriodic() {
        Drivetrain.loop();
    }
}
