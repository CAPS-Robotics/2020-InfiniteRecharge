package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controllers {
    private static XboxController driveController;
    private static XboxController subsystemController;

    public static void init() {
        driveController = new XboxController(0);
        subsystemController = new XboxController(1);
    }

    public static double getLeftYAxis(boolean isDriveController) {
        if(isDriveController) {
            return -driveController.getRawAxis(1);
        } else {
            return -subsystemController.getRawAxis(1);
        }
    }
    public static double getRightYAxis(boolean isDriveController) {
        if(isDriveController) {
            return -driveController.getRawAxis(5);
        } else {
            return -subsystemController.getRawAxis(5);
        }
    }
    public static double getLeftXAxis(boolean isDriveController) {
        if(isDriveController) {
            return driveController.getRawAxis(0);
        } else {
            return subsystemController.getRawAxis(0);
        }
    }
    public static double getRightXAxis(boolean isDriveController) {
        if(isDriveController) {
            return driveController.getRawAxis(4);
        } else {
            return subsystemController.getRawAxis(4);
        }
    }
}
