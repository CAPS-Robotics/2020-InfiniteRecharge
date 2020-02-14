package frc.robot.MotionProfoling;

public class Spline {
    double x0, y0, x1, y1, theta0, theta1;
    double ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy;
    double scale;

    public Spline(double x0, double y0, double x1, double y1, double theta0, double theta1) {
        this.x0 = x0;
        this.y0 = y0;
        this.x1 = x1;
        this.y1 = y1;
        this.theta0 = -theta0 + 90;
        this.theta1 = -theta1 + 90;

        scale = 1.2 * Math.sqrt(Math.pow(x1 - x0, 2) + Math.pow(y1 - y0, 2));
        calculateCoefficients();
    }

    public void calculateCoefficients() {
        ax = checkZero(-6 * x0 + 6d * x1 - 3d * Math.cos(Math.toRadians(theta0)) * scale - 3d * Math.cos(Math.toRadians(theta1)) * scale);
        bx = checkZero(15d * x0 - 15d * x1 + 8d * Math.cos(Math.toRadians(theta0)) * scale + 7d * Math.cos(Math.toRadians(theta1)) * scale);
        cx = checkZero(-10d * x0 + 10d * x1 - 6d * Math.cos(Math.toRadians(theta0)) * scale - 4d * Math.cos(Math.toRadians(theta1)) * scale);
        dx = 0d;
        ex = checkZero(Math.cos(Math.toRadians(theta0)) * scale);
        fx = checkZero(x0);

        ay = checkZero(-6 * y0 + 6 * y1 - 3 * Math.sin(Math.toRadians(theta0)) * scale - 3 * Math.sin(Math.toRadians(theta1)) * scale);
        by = checkZero(15 * y0 - 15 * y1 + 8 * Math.sin(Math.toRadians(theta0)) * scale + 7 * Math.sin(Math.toRadians(theta1)) * scale);
        cy = checkZero(-10 * y0 + 10 * y1 - 6 * Math.sin(Math.toRadians(theta0)) * scale - 4 * Math.sin(Math.toRadians(theta1)) * scale);
        dy = 0d;
        ey = checkZero(Math.sin(Math.toRadians(theta0)) * scale);
        fy = checkZero(y0);
    }

    private double checkZero(double d) {
        if(d <= 1e-10 && d >= -1e-10) return 0;
        return d;
    }

    public double getLeftPosX(double t) { return getXOffsetPos(t, true); }
    public double getLeftPosY(double t) { return getYOffsetPos(t, true); }
    public double getRightPosX(double t) { return getXOffsetPos(t, false); }
    public double getRightPosY(double t) { return getYOffsetPos(t, false); }

    private double getXOffsetPos(double t, boolean left) {
        return getX(t) + (Math.sqrt(Math.pow(VelocityProfile.WHEELBASE / 2, 2) - Math.pow(getYOffsetPos(t, left) - getY(t), 2))) * (left ? -1 : 1) * (getdy(t) > 0 ? 1 : -1);
    }
    private double getYOffsetPos(double t, boolean left) {
        if(Double.isInfinite(getNormalSlope(t))) return getY(t) + VelocityProfile.WHEELBASE / 2 * (left ? -1 : 1) * (t - 0.001 > 0 ? (getdx(t - 0.001) > 0 ? -1 : 1) : (getdx(t + 0.001) > 0 ? -1 : 1));
        return getY(t) + getNormalSlope(t) * (VelocityProfile.WHEELBASE / 2) * Math.sqrt(1 / (Math.pow(getNormalSlope(t), 2) + 1)) * (left ? -1 : 1) * (getdy(t) > 0 ? 1 : -1);
    }

    public double getNormalSlope(double t) {
        if(getdydx(t) == 0) return Double.POSITIVE_INFINITY;
        if(Double.isInfinite(getdydx(t))) return 0;
        return -1 / getdydx(t);
    }

    public double getX(double t) {
        return ax * Math.pow(t, 5) + bx * Math.pow(t, 4) + cx * Math.pow(t, 3) + dx * Math.pow(t, 2) + ex * Math.pow(t, 1) + fx;
    }
    public double getY(double t) {
        return ay * Math.pow(t, 5) + by * Math.pow(t, 4) + cy * Math.pow(t, 3) + dy * Math.pow(t, 2) + ey * Math.pow(t, 1) + fy;
    }
    public double getdx(double t) {
        return 5 * ax * Math.pow(t, 4) + 4 * bx * Math.pow(t, 3) + 3 * cx * Math.pow(t, 2) + 2 * dx * Math.pow(t, 1) + ex;
    }
    public double getddx(double t) {
        return 20 * ax * Math.pow(t, 3) + 12 * bx * Math.pow(t, 2) + 6 * cx * Math.pow(t, 1) + 2 * dx;
    }
    public double getdy(double t) {
        return 5 * ay * Math.pow(t, 4) + 4 * by * Math.pow(t, 3) + 3 * cy * Math.pow(t, 2) + 2 * dy * Math.pow(t, 1) + ey;
    }

    public double getddy(double t) {
        return 20 * ay * Math.pow(t, 3) + 12 * by * Math.pow(t, 2) + 6 * cy * Math.pow(t, 1) + 2 * dy;
    }
    public double getdydx(double t) {
        if(getdx(t) == 0) return Double.POSITIVE_INFINITY;
        return getdy(t) / getdx(t);
    }
    public double getd2ydx2(double t) {
        return (getdx(t) * getddy(t) - getdy(t) * getddx(t)) / Math.pow(getdx(t), 3);
    }
    public double getCurvature(double t) {
        return (getdx(t) * getddy(t) - getdy(t) * getddx(t)) / Math.pow(Math.pow(getdx(t), 2) + Math.pow(getdy(t), 2), 1.5);
    }
    public double getAngle(double t) {
        double angle = Math.toDegrees(Math.atan(getdydx(t)));
        angle = angle > 0 ? 90 - angle : (90 + angle) * -1;
        if(getdx(t) > 0 && getdy(t) < 0) {
            return 180 - Math.abs(angle);
        } else if(getdx(t) < 0 && getdy(t) < 0) {
            return -180 + Math.abs(angle);
        }
        return angle;
    }
}