package frc.robot.MotionProfiling;

public class LinearApproximation {
    double x, y, m;

    public LinearApproximation(double x0, double x1, double y0, double y1) {
        x = x0;
        y = y0;
        m = (y1 - y0) / (x1 - x0);
    }

    public double getT(double distance) {
        return m * (distance - x) + y;
    }
}
