package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

import org.firstinspires.ftc.teamcode.DriveEngine.MecunamDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Util.Controller;

@TeleOp
public class HardwareTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx a = hardwareMap.get(DcMotorEx.class, "arm");
        Controller con = new Controller(gamepad1);
        MecunamDrive drive = new MecunamDrive(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            if(con.rightTriggerHeld){
                a.setPower(-.5);
            }else if(con.leftTriggerHeld){
                a.setPower( .5);
            }else{
                a.setPower(0);
            }

            telemetry.addData("po", a.getCurrentPosition());

            con.update();
            telemetry.update();
        }

    }
}