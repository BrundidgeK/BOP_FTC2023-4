package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class twoWheelOdometry {
    public Vector2D position;

    public double x, y;

    public static final double INCHES_PER_REVOLUTION = ((1.88976377953 * Math.PI) / 2000); // Odom wheel Circumference(inches) / (Ticks Per Revolution of odom pods, google it)
    public static final double HORIZONTAL_DISTANCE_CENTER = 10, VERTICAL_DISTANCE_CENTER = 5; //Distance(inches) each odom wheel is from center respectively

    private int prevVert, prevHori;
    private DcMotor hori, vert;

    private BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    public double globalAngle, prevAngle;

    public twoWheelOdometry(HardwareMap hw){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hw.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        hori = hw.get(DcMotor.class, "perpendOdom");
        vert = hw.get(DcMotor.class, "parallelOdom");

        position = new Vector2D(0, 0,0);
        hori.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hori.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vert.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getX(){
        return vert.getCurrentPosition();
    }

    public double getY(){
        return hori.getCurrentPosition();
    }

    private void calculateChange(){
        double dh = ((hori.getCurrentPosition() - prevHori)),
                dv = ((vert.getCurrentPosition() - prevVert));

        double dx = (dv * Math.cos(Math.toRadians(globalAngle))) + (dh * Math.sin(Math.toRadians(globalAngle))),
                dy = (dh * Math.cos(Math.toRadians(globalAngle))) - (dv * Math.sin(Math.toRadians(globalAngle)));

        prevVert = vert.getCurrentPosition();
        prevHori = hori.getCurrentPosition();

        //dx *= Math.sin(Math.toRadians(position.angle));
        //dy *= Math.cos(Math.toRadians(position.angle));

        x += dx * INCHES_PER_REVOLUTION;
        y += dy * INCHES_PER_REVOLUTION;
    }

    private void updateHeading(){
        prevAngle = globalAngle;
        Orientation angles = imu.getAngularOrientation
                (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        if (globalAngle < -180)
            globalAngle += 360;
        else if (globalAngle > 180)
            globalAngle -= 360;

    }

    private double[] moveTowards(Vector2D diff) {
        double robotMovementAngle = Math.atan2(diff.y, diff.x), //angle of movement
            distance = Math.hypot(diff.x, diff.y),
            h = robotMovementAngle - globalAngle,
            x = distance * Math.cos(h),
            y = distance * Math.sin(h);

        double rotate = Range.clip(h, -.5, .5);

        double length = Math.hypot(x, y);
        if(length > 10){
            x /= length;
            y /= length;
        }

        return new double[]{
            x + y + rotate,
            x - y + rotate,
            x + y - rotate,
            x - y - rotate}
        ;
    }

    public void update(){
        updateHeading();
        calculateChange();
    }

}
