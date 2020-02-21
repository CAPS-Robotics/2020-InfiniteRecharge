package frc.robot.MotionProfiling;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class VelocityProfile {
    public static final double MAX_VELOCITY = 17.9;
    public static final double MAX_ACCELERATION = 12;
    public static final double WHEELBASE = 1.75;

    private static ArrayList<Point> points = new ArrayList<>();
    private static ArrayList<Spline> path = new ArrayList<>();
    private static ArrayList<LinearApproximation> approximations = new ArrayList<>();

    private static double pathDistance;
    private static double leftPathDistance;
    private static double rightPathDistance;

    private static double currentLeftDistance;
    private static double currentRightDistance;
    private static double currentLeftVelocity;
    private static double currentRightVelocity;
    private static double currentAngle;

    private static double pLeftVelocity;
    private static double pRightVelocity;
    public static double pT;
    private static boolean decelerating;
    private static int splineIndex;

    private static boolean backwards;

    public static void generatePath(boolean isBackwards) {
        backwards = isBackwards;
        setPath();
        calculateDistance();
    }

    public static void addWaypoint(double x, double y, double theta) { points.add(new Point(x, y, theta)); }

    private static void setPath() {
        for(int i = 0; i < points.size() - 1; i++) {
            path.add(new Spline(points.get(i), points.get(i + 1)));
        }
    }

    public static void reset() {
        points = new ArrayList<>();
        path = new ArrayList<>();
        approximations = new ArrayList<>();
        pLeftVelocity = 0;
        pRightVelocity = 0;
        pT = 0;
        decelerating = false;
        splineIndex = 0;
        currentLeftDistance = 0;
        currentRightDistance = 0;
    }

    private static void calculateDistance() {
        double distance = 0;
        double leftDistance = 0;
        double rightDistance = 0;

        for(int i = 0; i < path.size(); i++) {
            double pDistance = distance;
            double dt = 0.005;
            for(double t = 0; t <= 1; t += dt) {
                Spline spline = path.get(i);
                distance += Math.sqrt(Math.pow(spline.getdx(t), 2) + Math.pow(spline.getdy(t), 2)) * dt;
                leftDistance += Math.sqrt((Math.pow((spline.getLeftPosY(t) - spline.getLeftPosY(t + dt)) / (spline.getLeftPosX(t) - spline.getLeftPosX(t + dt)), 2) + 1)) * Math.abs(spline.getLeftPosX(t) - spline.getLeftPosX(t + dt));
                rightDistance += Math.sqrt((Math.pow((spline.getRightPosY(t) - spline.getRightPosY(t + dt)) / (spline.getRightPosX(t) - spline.getRightPosX(t + dt)), 2) + 1)) * Math.abs(spline.getRightPosX(t) - spline.getRightPosX(t + dt));
            }
            approximations.add(new LinearApproximation(pDistance, distance, i, i+1));
        }
        pathDistance = distance;
        leftPathDistance = leftDistance;
        rightPathDistance = rightDistance;
    }

    public static double getPathDistance() { return pathDistance; }

    public static void calculateVelocities(double distance) {
        if(approximations.get(splineIndex).getT(distance) >= splineIndex + 1 && splineIndex < (path.size() - 1)) {
            splineIndex++;
            pT = 1 - pT;
        }

        double t = approximations.get(splineIndex).getT(distance) - (int)approximations.get(splineIndex).getT(distance);
        Spline currentSpline = path.get(splineIndex);

        double dt = t - pT;
        double pDistance = (currentLeftDistance + currentRightDistance) / 2;
        double dLeftDistance = Math.sqrt((Math.pow((currentSpline.getLeftPosY(t) - currentSpline.getLeftPosY(t + dt)) / (currentSpline.getLeftPosX(t) - currentSpline.getLeftPosX(t + dt)), 2) + 1)) * Math.abs(currentSpline.getLeftPosX(t) - currentSpline.getLeftPosX(t + dt));
        double dRightDistance = Math.sqrt((Math.pow((currentSpline.getRightPosY(t) - currentSpline.getRightPosY(t + dt)) / (currentSpline.getRightPosX(t) - currentSpline.getRightPosX(t + dt)), 2) + 1)) * Math.abs(currentSpline.getRightPosX(t) - currentSpline.getRightPosX(t + dt));

        currentLeftDistance += dLeftDistance;
        currentRightDistance += dRightDistance;
        SmartDashboard.putNumber("dDistance", (currentLeftDistance + currentRightDistance) / 2 - pDistance);

        double leftVelocity;
        double rightVelocity;
        if(currentSpline.getdx(t) > 0 && currentSpline.getd2ydx2(t) > 0 || currentSpline.getdx(t) < 0 && currentSpline.getd2ydx2(t) < 0) {
            rightVelocity = Math.min(MAX_VELOCITY, calcMaxVelocity(pRightVelocity, dRightDistance));
            leftVelocity = calcInnerWheelVelocity(rightVelocity, currentSpline.getCurvature(t));
            if(leftVelocity > calcMaxVelocity(pLeftVelocity, dLeftDistance)) {
                leftVelocity = calcMaxVelocity(pLeftVelocity, dLeftDistance);
                rightVelocity = calcOuterWheelVelocity(leftVelocity, currentSpline.getCurvature(t));
            }
            decelerating = calcStoppingDistance(rightVelocity) >= rightPathDistance - currentRightDistance;
            if(decelerating) {
                rightVelocity = calcMinVelocity(pRightVelocity, dRightDistance);
                leftVelocity = calcInnerWheelVelocity(rightVelocity, currentSpline.getCurvature(t));
            }
        } else if (currentSpline.getdx(t) < 0 && currentSpline.getd2ydx2(t) > 0 || currentSpline.getdx(t) > 0 && currentSpline.getd2ydx2(t) < 0) {
            leftVelocity = Math.min(MAX_VELOCITY, calcMaxVelocity(pLeftVelocity, dLeftDistance));
            rightVelocity = calcInnerWheelVelocity(leftVelocity, currentSpline.getCurvature(t));
            if(rightVelocity > calcMaxVelocity(pRightVelocity, dRightDistance)) {
                rightVelocity = calcMaxVelocity(pRightVelocity, dRightDistance);
                leftVelocity = calcOuterWheelVelocity(rightVelocity, currentSpline.getCurvature(t));
            }
            decelerating = calcStoppingDistance(leftVelocity) >= leftPathDistance - currentLeftDistance;
            if(decelerating) {
                leftVelocity = calcMinVelocity(pLeftVelocity, dLeftDistance);
                rightVelocity = calcInnerWheelVelocity(leftVelocity, currentSpline.getCurvature(t));
            }
        } else {
            double maxLeftVelocity = calcMaxVelocity(pLeftVelocity, dLeftDistance);
            double maxRightVelocity = calcMaxVelocity(pRightVelocity, dRightDistance);
            double currentMaxVelocity = Math.min(maxLeftVelocity, maxRightVelocity);
            leftVelocity = Math.min(currentMaxVelocity, MAX_VELOCITY);
            rightVelocity = Math.min(currentMaxVelocity, MAX_VELOCITY);
            decelerating = calcStoppingDistance(rightVelocity) >= rightPathDistance - currentRightDistance;
            if(decelerating) {
                leftVelocity = calcMinVelocity(pLeftVelocity, dLeftDistance);
                rightVelocity = calcMinVelocity(pRightVelocity, dRightDistance);
            }
        }

        pLeftVelocity = leftVelocity;
        pRightVelocity = rightVelocity;
        pT = t;

        currentLeftVelocity = leftVelocity;
        currentRightVelocity = rightVelocity;
        currentAngle = currentSpline.getAngle(t);

        if(backwards) {
            double temp = currentLeftVelocity;
            currentLeftVelocity = -currentRightVelocity;
            currentRightVelocity = -temp;
            currentAngle += currentAngle > 0 ? -180 : 180;
        }
    }

    public static double getCurrentLeftVelocity() { return currentLeftVelocity; }
    public static double getCurrentRightVelocity() { return currentRightVelocity; }
    public static double getCurrentAngle() { return currentAngle; }

    private static double calcMaxVelocity(double pVelocity, double distance) {
        return Math.sqrt(Math.pow(pVelocity, 2) + 2 * MAX_ACCELERATION * distance);
    }
    private static double calcMinVelocity(double pVelocity, double distance) {
        return Math.sqrt(Math.pow(pVelocity, 2) + 2 * -MAX_ACCELERATION * distance);
    }
    private static double calcInnerWheelVelocity(double outerVelocity, double curvature) {
        double turningRadius = Math.abs(1 / curvature);
        return (2 * turningRadius * outerVelocity - WHEELBASE * outerVelocity) / (2 * turningRadius + WHEELBASE);
    }
    private static double calcOuterWheelVelocity(double innerVelocity, double curvature) {
        double turningRadius = Math.abs(1 / curvature);
        return (2 * turningRadius * innerVelocity + WHEELBASE * innerVelocity) / (2 * turningRadius - WHEELBASE);
    }

    private static double calcStoppingDistance(double velocity) {
        return Math.pow(velocity, 2) / (2 * MAX_ACCELERATION);
    }

    public static ArrayList<Spline> getPath() { return path; }
}