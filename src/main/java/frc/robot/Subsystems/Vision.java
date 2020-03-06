package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
        public static final double CAMERA_ANGLE = 20;
        public static final double CAMERA_HEIGHT = 1.45833333333333333333333;
        public static final double SHOOTING_HEIGHT = 8.1875;
        private static NetworkTable table;

        public static void init() {
                table = NetworkTableInstance.getDefault().getTable("limelight");
        }
        public static void loop() {

        }

        public static double getDistance() {
                return (SHOOTING_HEIGHT - CAMERA_HEIGHT) / (Math.tan(Math.toRadians(CAMERA_ANGLE + table.getEntry("ty").getDouble(0))));
        }

        public static double angleX() {
                return ( table.getEntry("tx").getDouble(0));
        }
}