package frc.robot.Autonomous;

import frc.robot.MotionProfiling.VelocityProfile;
import frc.robot.Subsystems.Drivetrain;

public class RightTrench {
    private static VelocityProfile autoProfile = new VelocityProfile();

    public static void initPath() {
        autoProfile.reset();
        autoProfile.addWaypoint(0, 0, 180);
        autoProfile.addWaypoint(6, -6, 180);
        autoProfile.generatePath(false);
    }

    public static void startPath() {
        Drivetrain.startProfile(autoProfile);
        autoProfile.startPath();
    }
}
