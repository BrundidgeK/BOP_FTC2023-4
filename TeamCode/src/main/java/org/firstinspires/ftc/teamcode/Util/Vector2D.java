package org.firstinspires.ftc.teamcode.Util;

public class Vector2D {
    public double x, y, angle;
    public Vector2D(double x, double y){
        this.x = x;
        this.y = y;

        angle = Math.toDegrees(Math.atan2(y, x));
    }
    public Vector2D(double x, double y, double theta){
        this.x = x;
        this.y = y;

        angle = Math.toDegrees(Math.atan2(y, x));
    }
}