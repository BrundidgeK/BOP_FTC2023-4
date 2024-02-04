package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm {
    private DcMotorEx armMotor;
    private ModernRoboticsI2cGyro gyro; // gyroscope

    public static final int INTAKE_POS = 0, SCORING_POS = 265;
    private double targetAngle = 0;
    private boolean goTarget;
    public static final int INCREMENT = 5; // Increment value
    public static final double DELAY_SECONDS = 0.2; // Delay in seconds

    public Arm(HardwareMap hardwareMap, LinearOpMode op) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        gyro.calibrate();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (op.opModeIsActive() && gyro.isCalibrating())  {
            op.telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
            op.telemetry.update();
            op.sleep(50);
        }

        op.telemetry.log().clear();
        op.telemetry.log().add("Gyro Calibrated. Press Start.");
        op.telemetry.clear();
        op.telemetry.update();
    }

    public void setPower(double power) {
        armMotor.setPower(power);
    }

    public void setPosition(double position, double power) {
        targetAngle = position;
        setPower(power);
        goTarget = true;

       /* armMotor.setTargetPosition(position);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        */
    }

    public void moveBucketIncrementally(boolean minus) {
        //setPosition(getCurrentPosition() + (minus ? -INCREMENT : INCREMENT));
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }

    public int getTarget() {
        return armMotor.getTargetPosition();
    }

    public double getHeading() {
        return gyro.getHeading();
    }

    public boolean isBusy(){
        return goTarget && Math.abs(targetAngle - getHeading()) >= 5;
    }

    public void  update() {
        if (goTarget && Math.abs(targetAngle - getHeading()) <= 5) {
            setPosition(getHeading(), 0);
            goTarget = false;
        }
    }
}
