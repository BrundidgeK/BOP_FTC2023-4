package org.firstinspires.ftc.teamcode.DriveEngine;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class

MecunamDrive {
    private double maxSpeed = 0.8;
    private double slowSpeed = 0.4;
    private double currentSpeed = maxSpeed;
    public static final String[] MOTORS_NAMES = {
            "Front Left",
            "Back Left",
            "Back Right",
            "Front Right"
    };
    private DcMotorEx[] motors = new DcMotorEx[4];

    public MecunamDrive(HardwareMap hw) {
        for (int i = 0; i < MOTORS_NAMES.length; i++) {
            motors[i] = hw.get(DcMotorEx.class, MOTORS_NAMES[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        currentSpeed = maxSpeed;
    }

    public void moveWithPower(double power) {
        for (DcMotorEx motor : motors) {
            motor.setPower(power * currentSpeed);
        }
    }

    public void moveWithPower(double fl, double bl, double br, double fr) {
        motors[0].setPower(fl * currentSpeed);
        motors[1].setPower(bl * currentSpeed);
        motors[2].setPower(br * currentSpeed);
        motors[3].setPower(fr * currentSpeed);
    }

    public void slowMode(boolean slow) {
        if (slow){
            currentSpeed = slowSpeed;
        } else{
            currentSpeed = maxSpeed;
        }
    }

    public boolean slowModeActivated() {
        return currentSpeed == slowSpeed;
    }
}
