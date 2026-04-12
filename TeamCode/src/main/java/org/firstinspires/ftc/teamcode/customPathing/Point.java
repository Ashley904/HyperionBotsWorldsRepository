package org.firstinspires.ftc.teamcode.customPathing;

public class Point {
    private double x = 0.0, y = 0.0, heading = 0.0;

    public Point(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }




    public void setPoint(double x, double y, double heading ){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }




    public double getX() { return x;}
    public double getY() { return y;}
    public double getHeading() { return heading;}
}
