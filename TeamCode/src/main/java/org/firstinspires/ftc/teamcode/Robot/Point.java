package org.firstinspires.ftc.teamcode.Robot;

public class Point {

    public double x;
    public double y;
    public double angle;
    public double dynamicAngle = 0;
    public boolean hasAngle = false;
    boolean hasDynamicAngle = false;

    public double power;
    public boolean hasPower = false;

    public static enum AngleType{
        DIRECT,
        DYNAMIC
    }

    public Point() {

    }

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double angle) {
        hasAngle = true;
        this.angle = angle;
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double angle, AngleType angleType) {
        if (angleType==AngleType.DIRECT) {
            hasAngle = true;
            this.angle = angle;
        } else if (angleType==AngleType.DYNAMIC) {
            hasDynamicAngle = true;
            dynamicAngle = angle;
        }
        this.x = x;
        this.y = y;
    }

    public Point(double x, double y, double angle, AngleType angleType, double power) {
        if (angleType==AngleType.DIRECT) {
            hasAngle = true;
            this.angle = angle;
        } else if (angleType==AngleType.DYNAMIC) {
            hasDynamicAngle = true;
            dynamicAngle = angle;
        }
        this.power = power;
        hasPower = true;
        this.x = x;
        this.y = y;
    }

    public void setX(double x) {
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    public void setPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public void setPoint(double x, double y, double angle) {
        hasAngle = true;
        this.angle = angle;
        this.x = x;
        this.y = y;
    }
    public void setPoint(Point point) {
        this.x = point.x;
        this.y = point.y;
    }

    public Point add(Point point) {
        Point newPoint = new Point(this.x+point.x, this.y+point.y);
        return newPoint;
    }

    public Point subtract(Point point) {
        Point newPoint = new Point(this.x-point.x, this.y-point.y);
        return newPoint;
    }

    public boolean isBetween(Point point1, Point point2) {
        boolean xIsBetween = false;
        boolean yIsBetween = false;
        if ((x <= point1.x && x >= point2.x) || (x >= point1.x && x <= point2.x)) {
            xIsBetween = true;
        }
        if ((y <= point1.y && y >= point2.y) || (y >= point1.y && y <= point2.y)) {
            yIsBetween = true;
        }
        if (xIsBetween && yIsBetween) {
            return true;
        }
        else {
            return false;
        }
    }

    public double distanceTo(Point point) {
        Point relativePoint = point.subtract(this);
        return Math.sqrt(Math.pow(relativePoint.x, 2) + Math.pow(relativePoint.y, 2));
    }
}
