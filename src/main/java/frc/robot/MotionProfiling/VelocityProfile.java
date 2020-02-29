package frc.robot.MotionProfiling;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Drivetrain;

import java.util.ArrayList;

public class VelocityProfile {
    public static final double MAX_VELOCITY = 17.9;
    public static final double MAX_ACCELERATION = 12;
    public static final double WHEELBASE = 1.75;

    private Timer timer = new Timer();
    private ArrayList<Point> points = new ArrayList<>();
    private ArrayList<Spline> path = new ArrayList<>();
    private ArrayList<LinearApproximation> approximations = new ArrayList<>();

    private double pathDistance;
    private double leftPathDistance;
    private double rightPathDistance;

    private double currentDistance;
    private double currentLeftDistance;
    private double currentRightDistance;
    private double currentLeftVelocity;
    private double currentRightVelocity;
    private double currentAngle;

    private double pLeftDistance;
    private double pRightDistance;
    private double pLeftVelocity;
    private double pRightVelocity;
    private double dLeftVelocity;
    private double dRightVelocity;
    public double pT;
    private double pTime;
    private boolean decelerating;
    private int splineIndex;

    private boolean backwards;

    public void generatePath(boolean isBackwards) {
        backwards = isBackwards;
        setPath();
        calculateDistance();
    }

    public void addWaypoint(double x, double y, double theta) { points.add(new Point(x, y, theta)); }

    private void setPath() {
        for(int i = 0; i < points.size() - 1; i++) {
            path.add(new Spline(points.get(i), points.get(i + 1)));
        }
    }

    public void startPath() {
        timer.start();
        Drivetrain.resetEncoders();
    }
    public void reset() {
        points = new ArrayList<>();
        path = new ArrayList<>();
        approximations = new ArrayList<>();
        pLeftDistance = 0;
        pRightDistance = 0;
        pLeftVelocity = 0;
        pRightVelocity = 0;
        pT = -0.05;
        pTime = 0;
        pathDistance = 0;
        decelerating = false;
        splineIndex = 0;
        currentDistance = 0;
        currentLeftDistance = 0;
        currentRightDistance = 0;
    }

    private void calculateDistance() {
        double distance = 0;
        double leftDistance = 0;
        double rightDistance = 0;

        for(int i = 0; i < path.size(); i++) {
            double pDistance = distance;
            double dt = 0.1 / path.get(i).getLinearDistance();
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

    public double getPathDistance() { return pathDistance; }

    public boolean calculateVelocities() {
        double time = timer.get();
        double leftDistance = Drivetrain.getLeftDistance() * (backwards ? -1 : 1);
        double rightDistance = Drivetrain.getRightDistance() * (backwards ? -1 : 1);
        double distance = (leftDistance + rightDistance) / 2;

        if(approximations.get(splineIndex).getT(distance) >= splineIndex + 1 && splineIndex < path.size() - 1) {
            splineIndex++;
            pT = 1 - pT;
        }

        double t = approximations.get(splineIndex).getT(distance) % 1;
        SmartDashboard.putNumber("t", t);
        Spline currentSpline = path.get(splineIndex);

        double dTime = time - pTime;

        double dLeftDistance = leftDistance - pLeftDistance;
        double dRightDistance = rightDistance - pRightDistance;

        currentLeftDistance += dLeftDistance;
        currentRightDistance += dRightDistance;
        pLeftDistance = currentLeftDistance;
        pRightDistance = currentRightDistance;

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
        double currentVelocity = (currentLeftVelocity + currentRightVelocity);
        if(currentVelocity <= 0 && !backwards || currentVelocity >= 0 && backwards) return true;

        SmartDashboard.putNumber("Target angle", currentAngle);
        return false;
    }

    public double getCurrentLeftVelocity() { return currentLeftVelocity; }
    public double getCurrentRightVelocity() { return currentRightVelocity; }
    public double getCurrentAngle() { return currentAngle; }

    private double calcMaxVelocity(double pVelocity, double time) {
        return pVelocity + MAX_ACCELERATION * time;
    }
    private double calcMinVelocity(double pVelocity, double time) {
        return pVelocity - MAX_ACCELERATION * time;
    }
    private double calcInnerWheelVelocity(double outerVelocity, double curvature) {
        double turningRadius = Math.abs(1 / curvature);
        return (2 * turningRadius * outerVelocity - WHEELBASE * outerVelocity) / (2 * turningRadius + WHEELBASE);
    }
    private double calcOuterWheelVelocity(double innerVelocity, double curvature) {
        double turningRadius = Math.abs(1 / curvature);
        return (2 * turningRadius * innerVelocity + WHEELBASE * innerVelocity) / (2 * turningRadius - WHEELBASE);
    }

    private double calcStoppingDistance(double velocity) {
        return Math.pow(velocity, 2) / (2 * MAX_ACCELERATION);
    }

    public ArrayList<Spline> getPath() { return path; }
}