package frc.robot.MotionProfiling;

public class Point {
    private double x, y, theta;

    public Point(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = 90 - theta;
    }

    public double getX() { return x; }
    public double getY() { return y; }
    public double getTheta() { return theta; }
}