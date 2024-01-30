package org.firstinspires.ftc.teamcode.Subsystems;/*package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Deprecated

public class

ArmAndClaw {
    private Servo leftArmServo;
    private Servo rightArmServo;

    private Servo claw;


    public static final double A_OPEN = 1, A_CLOSE = 0;//Positions for arm
    public static final double C_OPEN = 1, C_CLOSE = 0;//Positions for claw

    public ArmAndClaw(HardwareMap hwMap) {
        //leftArmServo = hwMap.get(Servo.class, "servoarm1");
        rightArmServo = hwMap.get(Servo.class, "servoarm1");
        claw = hwMap.get(Servo.class, "servoclaw");
    }

    public void setArmPosition(double position) {
        //leftArmServo.setPosition(position);
        rightArmServo.setPosition(position);
    }
    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public double getArmPosition(){
        return rightArmServo.getPosition();
    }

    public double getClawPosition(){
        return claw.getPosition();
    }
}*/