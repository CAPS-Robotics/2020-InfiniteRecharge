package frc.team2410.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.team2410.robot.Subsystems.Drivetrain;
import frc.team2410.robot.Subsystems.Intake;
import frc.team2410.robot.Subsystems.Shooter;

public class Robot extends TimedRobot {
    @Override
    public void robotInit() {
        Drivetrain.init();
        Intake.init();
        Controllers.init();
        Shooter.init();
    }

    @Override
    public void teleopPeriodic() {
        Drivetrain.loop();
        Intake.loop();
        Shooter.loop();
    }
}
