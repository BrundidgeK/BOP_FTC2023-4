package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//import UtilityClasses.Location;

public class Lift {
    private DcMotorEx lift;

    private boolean braking;
    public boolean stopZero = true;

    private TouchSensor limitSwitch;

    private double powerDecrease = .3;

    public Lift(HardwareMap hardwareMap) {
        lift = hardwareMap.get(DcMotorEx.class, "llift");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
       // limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroLift();
    }

    public void setPower(double power) {
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(getPosition() <= 0 && power < 0 && stopZero){
            power = 0;
        } //else if(getPosition() >= 13000 && power >0){
        //    power = 0;
        //}

        if(power < 0)
            power *= powerDecrease;

        lift.setPower(power);
    }

    public void setPosition(int position, double power) {
        lift.setTargetPosition(position);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //if(position < liftMotor.getCurrentPosition()){
        //    power /= powerDecrease;
        //}
        lift.setPower(power);
        braking = false;
    }

    public int getPosition() {
        return lift.getCurrentPosition();
    }

    public double getCurrent() {
        return lift.getCurrent(CurrentUnit.AMPS);
    }

    public void brake() {
        if(!braking) {
            lift.setTargetPosition(lift.getCurrentPosition());
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(1);
            braking = true;
        }
    }

    public void zeroLift() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void update() {
        /*if (limitSwitch.isPressed()) {
            zeroLift();
            if(getPower() < 0){
                setPower(0);
            }
        }*/

        if(getPower() == 0 && stopZero && getPosition() < 0){
            zeroLift();
        }

        if(Math.abs(getTarget() - getPosition()) <= 50){
        //    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    public boolean isPressed(){
        return limitSwitch.isPressed();
    }

    public boolean isBusy(){
        return lift.isBusy();
    }

    public boolean closeToTarget(){
        return Math.abs(lift.getTargetPosition()- lift.getCurrentPosition()) <= 300;
    }

    public void addTargetClearance(){
        lift.setTargetPosition(lift.getTargetPosition() + 300);
    }

    public DcMotor.RunMode getMode(){
        return lift.getMode();
    }

    public double getPower() {
        return lift.getPower();
    }

    public int getTarget() {
        return lift.getTargetPosition();
    }

}