package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm {
    private DcMotorEx armMotor;
    private IMU imu; // gyroscope IMU

    public static final int DOWN = 0, SCORING = 0;
    public static final int INCREMENT = 5; // Increment value
    public static final double DELAY_SECONDS = 0.2; // Delay in seconds

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        imu = hardwareMap.get(IMU.class, "gyro");

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power) {
        armMotor.setPower(power);
    }

    public void setPosition(int position) {
        armMotor.setTargetPosition(position);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveBucketIncrementally(boolean minus) {
        setPosition(getCurrentPosition() + (minus ? -INCREMENT : INCREMENT));
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public int getTarget() {
        return armMotor.getTargetPosition();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void update() {
        if (Math.abs(getTarget() - getCurrentPosition()) <= 15) {
            setPosition(getCurrentPosition());
        }
    }
}
