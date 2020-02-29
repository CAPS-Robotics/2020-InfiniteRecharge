package frc.robot.Autonomous;

import frc.robot.MotionProfiling.VelocityProfile;
import frc.robot.Subsystems.Drivetrain;

public class Autonomous {
    public static enum AUTO_STATIONS {
        RIGHT_TRENCH
    }
    private static AUTO_STATIONS auto;

    public static void init() {
        RightTrench.initPath();
    }

    public static void loop() {
    }

    public static void setAuto(AUTO_STATIONS currentAuto) {
        auto = currentAuto;

        switch (auto) {
            case RIGHT_TRENCH:
                RightTrench.startPath();
        }
    }
}
