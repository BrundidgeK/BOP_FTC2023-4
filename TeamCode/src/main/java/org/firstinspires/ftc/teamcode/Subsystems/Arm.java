package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    private DcMotorEx armMotor;

    // Constants for arm positions and movement
    public static final int DOWN = 0, SCORING = 0;
    public static final int INCREMENT = 5; // Increment value
    public static final double DELAY_SECONDS = 0.2; // Delay in seconds

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");

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

    public void update() {
        if (Math.abs(getTarget() - getCurrentPosition()) <= 15) {
            setPosition(getCurrentPosition());
        }
    }
}
