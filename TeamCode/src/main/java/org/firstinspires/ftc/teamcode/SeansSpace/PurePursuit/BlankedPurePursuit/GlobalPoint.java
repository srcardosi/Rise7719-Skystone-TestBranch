package org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.BlankedPurePursuit;

import org.firstinspires.ftc.teamcode.SeansSpace.PurePursuit.MathElements.Point;

/**
 * Created by Sean Cardosi on 2019-12-23.
 */
public class GlobalPoint {

    public double x;
    public double y;
    public double heading;

    public GlobalPoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public GlobalPoint(GlobalPoint point) {
        x = point.x;
        y = point.y;
        heading = point.heading;
    }
    public Point toPoint() {
        return new Point(x,y);
    }
    public void setPoint(Point point) {
        x = point.x;
        y = point.y;
    }
}
