package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {
        public static final double CAMERA_ANGLE = 20;
        public static final double CAMERA_HEIGHT = 8;
        public static final double SHOOTING_HEIGHT = 98.25;
        private static NetworkTable table;

        private static boolean autoRotating;

        public static void init() {
                table = NetworkTableInstance.getDefault().getTable("limelight");
                autoRotating = false;
        }
        public static void loop() {
                if(autoRotating) {
                        turnToShoot();
                }
                SmartDashboard.putBoolean("Auto rotate", autoRotating);
        }

        public static double getDistance() {
                return (SHOOTING_HEIGHT - CAMERA_HEIGHT) / (Math.tan(Math.toRadians(CAMERA_ANGLE + table.getEntry("ty").getDouble(0))));
        }

        public static double angleX() {
                return ( table.getEntry("tx").getDouble(0));
        }

        public static void turnToShoot() {
                Drivetrain.setGyroHeading(Drivetrain.getHeading() + angleX());
                if(Drivetrain.atAngle()) autoRotating = false;
        }
}