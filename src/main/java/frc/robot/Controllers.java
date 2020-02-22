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
    public static boolean getLeftJoyButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(9);
        else return subsystemController.getRawButton(9);
    }
    public static boolean getRightJoyButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(10);
        else return subsystemController.getRawButton(10);
    }

    public static double getLeftTrigger(boolean isDriveController) {
        if (isDriveController) return driveController.getRawAxis(2);
        else return subsystemController.getRawAxis(2);
    }
    public static double getRightTrigger(boolean isDriveController) {
        if (isDriveController) return driveController.getRawAxis(3);
        else return subsystemController.getRawAxis(3);
    }
    public static boolean getLeftStartButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(7);
        else return subsystemController.getRawButton(7);
    }
    public static boolean getRightStartButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(8);
        else return subsystemController.getRawButton(8);
    }

    public static boolean getPOVUp(boolean isDriveController) {
        if(isDriveController) return driveController.getPOV() == 0;
        else return subsystemController.getPOV() == 0;
    }
    public static boolean getPOVDown(boolean isDriveController) {
        if(isDriveController) return driveController.getPOV() == 180;
        else return subsystemController.getPOV() == 180;
    }
    public static boolean getPOVLeft(boolean isDriveController) {
        if(isDriveController) return driveController.getPOV() == 270;
        else return subsystemController.getPOV() == 270;
    }
    public static boolean getPOVRight(boolean isDriveController) {
        if(isDriveController) return driveController.getPOV() == 90;
        else return subsystemController.getPOV() == 90;
    }

    public static boolean getAButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(1);
        else return subsystemController.getRawButton(1);
    }
    public static boolean getBButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(2);
        else return subsystemController.getRawButton(2);
    }
    public static boolean getXButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(3);
        else return subsystemController.getRawButton(3);
    }
    public static boolean getYButton(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(4);
        else return subsystemController.getRawButton(4);
    }
    public static boolean getLeftBumper(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(5);
        else return subsystemController.getRawButton(5);
    }
    public static boolean getRightBumper(boolean isDriveController) {
        if(isDriveController) return driveController.getRawButton(6);
        else return subsystemController.getRawButton(6);
    }

    public static double getDeadzone(double joystick, double deadzone) {
        if(Math.abs(joystick) < deadzone) return 0;
        return (Math.abs(joystick) - deadzone) / (1 - deadzone) * Math.abs(joystick) / joystick;
    }
}
