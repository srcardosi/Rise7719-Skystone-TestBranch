package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.PurePursuitGeometry;

import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;

/**
 * Created by Sean Cardosi.
 * CurvePoint is a custom Point class.
 */
public class CurvePoint {

    public double x;
    public double y;
    public double moveSpeed;
    public double turnSpeed;
    public double followDistance;

    public CurvePoint(double x, double y, double moveSpeed, double turnSpeed, double followDistance) {

        this.x = x;
        this.y = y;
        this.moveSpeed = moveSpeed;
        this.turnSpeed = turnSpeed;
        this.followDistance = followDistance;
    }

    public CurvePoint(CurvePoint thisPoint) {

        x = thisPoint.x;
        y = thisPoint.y;
        moveSpeed = thisPoint.moveSpeed;
        turnSpeed = thisPoint.turnSpeed;
        followDistance = thisPoint.followDistance;
    }

    public Point toPoint() {
        return new Point(x,y);
    }

    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
    }
}
