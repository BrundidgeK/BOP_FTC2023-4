package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Suspension {

    private DcMotorEx susMotor;

    public boolean braking;

    private Servo leftHook, rightHook;
    public static final int OPEN = 1, CLOSE = 0;

    public Suspension(HardwareMap hw){
        susMotor = hw.get(DcMotorEx.class, "suspend");
        susMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        susMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftHook = hw.get(Servo.class, "leftHook");
        rightHook = hw.get(Servo.class, "rightHook");
    }

    public void setPower(double power) {
        susMotor.setPower(power);
        if (power != 0) {
            susMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public int getCurrentPosition(){
       return susMotor.getCurrentPosition();
    }

    public void update(){
        /*
        if (braking && susMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
            susMotor.setPower(1);
            susMotor.setTargetPosition(sus


            0Motor.getCurrentPosition());
            susMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

         */

        if (susMotor.getPower() < 0 && getCurrentPosition() <= 0) {

            setPower(0);
            braking= true;
        }
    }

    public void setHookPostion(double position){
        leftHook.setPosition(position);
        rightHook.setPosition(1-position);
    }
    public double getHookPosition(){
        return leftHook.getPosition();
    }
}