package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.twoWheelOdometry;

@TeleOp
public class OdomTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        twoWheelOdometry odom = new twoWheelOdometry(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            odom.update();
            telemetry.addData("x", odom.getX());
            telemetry.addData("x", odom.getY());
            telemetry.addData("Pos", odom.x + ", " + odom.y + ", " + odom.globalAngle);
            telemetry.update();
        }
    }
}
