package org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.RISEDogeCV.RISEMath;

import org.firstinspires.ftc.teamcode.SeansSpace.DogeCV.RISEDogeCV.RISEHardware.RISEHardware;

/**
 * @Author Sean Cardosi
 * @Date 11/14/19
 */

public class RISEVector implements RISEHardware {
    private double x;
    private double y;
    private String name;
    public RISEVector(double x, double y, String name) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

    public RISEVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public RISEPoint toPoint() {
        return new RISEPoint(getX(), getY());
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public RISEVector unitVector () {
        return new RISEVector(getX()/getMagnitude(), getY()/getMagnitude());
    }

    public static RISEVector multiply(double scalar, RISEVector v) {
        return new RISEVector(v.getX() * scalar, v.getY() * scalar);
    }

    public double getMagnitude () {
        return Math.hypot(getX(), getY());
    }

    public double dotProduct(RISEVector v) {
        return (this.getX() * v.getX()) + (this.getY() * v.getY());
    }

    public double angleRad(RISEVector v) {
        return (Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
    }
    public double angleDeg(RISEVector v) {
        double deg = Math.toDegrees(Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
        if (Double.isNaN(deg)) return 0;
        return deg;
    }

    public double distanceToVector(RISEVector point) {
        return Math.hypot(point.getX() - getX(), point.getY() - getY());
    }

    public boolean equal(double radius, RISEVector v) {
        return distanceToVector(v) < radius;
    }

    public RISEVector displacement(RISEVector v) {
        return new RISEVector(v.getX() - getX(), v.getY() - getY());
    }

    public RISEVector projectOnTo(RISEVector v) {
        return multiply(dotProduct(v) / (v.getMagnitude() * v.getMagnitude()), v);
    }

    public double getDirection () {
        return Math.toDegrees(Math.atan(getY() / getX()));
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "X: " + getX(),
                "Y: " + getY(),
                "Direction: " + getDirection()
        };
    }
}