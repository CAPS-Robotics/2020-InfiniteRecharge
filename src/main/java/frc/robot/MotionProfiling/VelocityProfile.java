package frc.robot.MotionProfiling;

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

    private static double currentDistance;
    private static double currentLeftDistance;
    private static double currentRightDistance;
    private static double currentLeftVelocity;
    private static double currentRightVelocity;
    private static double currentAngle;

    private static double pLeftVelocity;
    private static double pRightVelocity;
    private static double dLeftVelocity;
    private static double dRightVelocity;
    public static double pT;
    private static double pTime;
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
        pT = -0.01;
        pTime = 0;
        pathDistance = 0;
        decelerating = false;
        splineIndex = 0;
        currentDistance = 0;
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

    public static boolean calculateVelocities(double time, double distance) {
        if(approximations.get(splineIndex).getT(currentDistance) >= splineIndex + 1) {
            splineIndex++;
            pT = 1 - pT;
            if(splineIndex == path.size()) return true;
        }

        double t = approximations.get(splineIndex).getT(distance) - (int)approximations.get(splineIndex).getT(currentDistance);
        Spline currentSpline = path.get(splineIndex);

        double dt = t - pT;
        double dTime = time - pTime;

        double dLeftDistance = Math.sqrt((Math.pow((currentSpline.getLeftPosY(t) - currentSpline.getLeftPosY(t + dt)) / (currentSpline.getLeftPosX(t) - currentSpline.getLeftPosX(t + dt)), 2) + 1)) * Math.abs(currentSpline.getLeftPosX(t) - currentSpline.getLeftPosX(t + dt));
        double dRightDistance = Math.sqrt((Math.pow((currentSpline.getRightPosY(t) - currentSpline.getRightPosY(t + dt)) / (currentSpline.getRightPosX(t) - currentSpline.getRightPosX(t + dt)), 2) + 1)) * Math.abs(currentSpline.getRightPosX(t) - currentSpline.getRightPosX(t + dt));

        currentLeftDistance += dLeftDistance;
        currentRightDistance += dRightDistance;
        System.out.println(currentRightDistance);

        double leftVelocity;
        double rightVelocity;
        if(currentSpline.getdx(t) > 0 && currentSpline.getd2ydx2(t) > 0 || currentSpline.getdx(t) < 0 && currentSpline.getd2ydx2(t) < 0) {
            rightVelocity = Math.min(MAX_VELOCITY, calcMaxVelocity(pRightVelocity, dTime));
            leftVelocity = calcInnerWheelVelocity(rightVelocity, currentSpline.getCurvature(t));
            if(leftVelocity > calcMaxVelocity(pLeftVelocity, dTime)) {
                leftVelocity = calcMaxVelocity(pLeftVelocity, dTime);
                rightVelocity = calcOuterWheelVelocity(leftVelocity, currentSpline.getCurvature(t));
            }
            decelerating = calcStoppingDistance(rightVelocity) >= rightPathDistance - currentRightDistance;
            if(decelerating) {
                rightVelocity = calcMinVelocity(pRightVelocity, dTime);
                leftVelocity = calcInnerWheelVelocity(rightVelocity, currentSpline.getCurvature(t));
            }
        } else if (currentSpline.getdx(t) < 0 && currentSpline.getd2ydx2(t) > 0 || currentSpline.getdx(t) > 0 && currentSpline.getd2ydx2(t) < 0) {
            leftVelocity = Math.min(MAX_VELOCITY, calcMaxVelocity(pLeftVelocity, dTime));
            rightVelocity = calcInnerWheelVelocity(leftVelocity, currentSpline.getCurvature(t));
            if(rightVelocity > calcMaxVelocity(pRightVelocity, dTime)) {
                rightVelocity = calcMaxVelocity(pRightVelocity, dTime);
                leftVelocity = calcOuterWheelVelocity(rightVelocity, currentSpline.getCurvature(t));
            }
            decelerating = calcStoppingDistance(leftVelocity) >= leftPathDistance - currentLeftDistance;
            if(decelerating) {
                leftVelocity = calcMinVelocity(pLeftVelocity, dTime);
                rightVelocity = calcInnerWheelVelocity(leftVelocity, currentSpline.getCurvature(t));
            }
        } else {
            double maxLeftVelocity = calcMaxVelocity(pLeftVelocity, dTime);
            double maxRightVelocity = calcMaxVelocity(pRightVelocity, dTime);
            double currentMaxVelocity = Math.min(maxLeftVelocity, maxRightVelocity);
            leftVelocity = Math.min(currentMaxVelocity, MAX_VELOCITY);
            rightVelocity = Math.min(currentMaxVelocity, MAX_VELOCITY);
            decelerating = calcStoppingDistance(rightVelocity) >= rightPathDistance - currentRightDistance;
            if(decelerating) {
                leftVelocity = calcMinVelocity(pLeftVelocity, dTime);
                rightVelocity = calcMinVelocity(pRightVelocity, dTime);
            }
        }

        dLeftVelocity = leftVelocity - pLeftVelocity;
        dRightVelocity = rightVelocity - pRightVelocity;
        pLeftVelocity = leftVelocity;
        pRightVelocity = rightVelocity;
        pT = t;
        pTime = time;

        currentLeftVelocity = leftVelocity;
        currentRightVelocity = rightVelocity;
        currentAngle = currentSpline.getAngle(t);

        if(backwards) {
            double temp = currentLeftVelocity;
            currentLeftVelocity = -currentRightVelocity;
            currentRightVelocity = -temp;
            currentAngle += currentAngle > 0 ? -180 : 180;
        }

        return false;
    }

    public static double getCurrentLeftVelocity() { return currentLeftVelocity; }
    public static double getCurrentRightVelocity() { return currentRightVelocity; }
    public static double getCurrentAngle() { return currentAngle; }

    private static double calcMaxVelocity(double pVelocity, double time) {
        return pVelocity + MAX_ACCELERATION * time;
    }
    private static double calcMinVelocity(double pVelocity, double time) {
        return pVelocity - MAX_ACCELERATION * time;
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