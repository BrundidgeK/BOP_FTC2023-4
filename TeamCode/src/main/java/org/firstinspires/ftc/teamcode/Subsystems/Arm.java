package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    /*
    private Servo leftArmServo;
    private Servo rightArmServo;

    public static final double A_OPEN = 1, A_CLOSE = 0;//Positions for arm

    public Arm     (HardwareMap hwMap) {
        leftArmServo = hwMap.get(Servo.class, "servoarml");
        rightArmServo = hwMap.get(Servo.class, "servoarmr");
    }

    public void setArmPosition(double position) {
        leftArmServo.setPosition(position);
        rightArmServo.setPosition(position);
    }

    public double getArmPosition(){
        return rightArmServo.getPosition();
    }

     */
    private DcMotorEx armMotor;

    public static final int DOWN = 0, SCORING = 0;
    public static final int INCREMENT = 5; // increment value
    public static final double DELAY_SECONDS = 0.2;// delay in seconds

    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setPower(power);
    }

    public void setPosition(int position){
        armMotor.setTargetPosition(position);
        armMotor.setPower(1);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveBucketIncrementally(boolean minus) {
            setPosition(getCurrentPosition() + (minus ? -INCREMENT : INCREMENT));
    }

    private void sleepSeconds(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public int getCurrentPosition(){
        return armMotor.getCurrentPosition();
    }

    public int getTarget(){
        return armMotor.getTargetPosition();
    }

    public void update(){
        //Stops when in clearance
        if(Math.abs(getTarget() - getCurrentPosition()) <= 15){
            setPosition(getCurrentPosition());
        }
    }
}