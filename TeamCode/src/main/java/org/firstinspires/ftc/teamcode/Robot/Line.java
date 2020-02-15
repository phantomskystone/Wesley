package org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import static java.lang.Math.*;

public class Line {

    double intercept;
    double slope;

    public static double vertical = 100000;


    public Line(double slope, double intercept) {
        this.slope = slope;
        this.intercept = intercept;
    }
    public Line(Point point1, Point point2) {
        if(point1.x == point2.x) {
            point2.x+=0.1;
            slope = (point2.y - point1.y) / (point2.x - point1.x);
        } else {
            slope = (point2.y - point1.y) / (point2.x - point1.x);
        }
        intercept = point1.y - (point1.x * slope);
    }
    public Line(Point point, double slope) {
        this.slope = slope;
        intercept = point.y - (point.x * slope);
    }

    public void setLine(double slope, double intercept) {
        this.slope = slope;
        this.intercept = intercept;
    }
    public void setLine(Point point1, Point point2) {
        if(point1.x == point2.x) {
            point2.x+=0.1;
            slope = (point2.y - point1.y) / (point2.x - point1.x);
        } else {
            slope = (point2.y - point1.y) / (point2.x - point1.x);
        }
        intercept = point1.y - (point1.x * slope);
    }
    public void setLine(Point point, double slope) {
        this.slope = slope;
        intercept = point.y - (point.x * slope);
    }

    public Line perpendicularThrough(Point point) {
        Line newLine;

        if (slope != 0 && slope != vertical) {
            newLine = new Line(point, -1 / slope);
        } else if (slope == 0) {
            newLine = new Line(point, vertical);
        } else if (slope == vertical) {
            newLine = new Line(point, 0);
        } else {
            newLine = null;
        }

        return newLine;
    }

    public Point intersection(Line line) {
        /*y = ax + b;
        y = cx + d;
        (ax + b) - (cx + d) = 0;
        ax + b - cx - d = 0;
        ax - cx = d - b
        x(a - c) = d - b
        x = (d - b) / (a - c)
         */
        if(this.slope != line.slope) {
            double x;
            double y;

            x = (intercept - line.intercept) / (line.slope - slope);
            y = slope * x + intercept;

            return new Point(x,y);

        } else {
            return null;
        }
    }

    public Point closestToPoint(Point point) {
        return intersection(perpendicularThrough(point));
    }

    //x=-2mb+- 2sqrt(-b2+r2m2+r2)  /  2m2+2

    public ArrayList<Point> pointAtDistance (Point point, Point towards, double radius) {
        ArrayList<Point> intersections = new ArrayList<>();

        Point opt1 = new Point();
        Point opt2 = new Point();

        double a = pow(slope,2)+1;
        double b = 2*slope*intercept;
        double c = pow(intercept,2)-pow(radius,2);

        opt1.setX((-b+sqrt(pow(b,2)-(4*a*c)))/(2*a));
        opt1.setY(intercept+slope*opt1.x);

        opt2.setX((-b-sqrt(pow(b,2)-(4*a*c)))/(2*a));
        opt2.setY(intercept+slope*opt2.x);

        intersections.add(opt1);
        intersections.add(opt2);

        return intersections;
    }
}
