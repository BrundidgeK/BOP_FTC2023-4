package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Intake {
    public static final double INTAKING = .65, OUTTAKING = -.65;



    public DcMotorEx intakeMotor;

    public Intake (HardwareMap hw) {
        intakeMotor = hw.get(DcMotorEx.class, "Intake");
    }

    public void setPower(double power){
        intakeMotor.setPower(power);
        if(power >= 0){
        }

    /*public void brake(){
        setPower(0);
    }*/
    }

    public void update() {

    }
}
