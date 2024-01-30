package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Util.Controller;

@Autonomous
public class RedParkAuto extends LinearOpMode {

    private boolean left;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //ArmAndClaw arm = new ArmAndClaw(hardwareMap);
        Controller con1 = new Controller(gamepad1);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(12.5, -57.5), Math.toRadians(90)) //Goes to Backboard
                .strafeRight(96).build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(12.5, -57.5), Math.toRadians(90)) //Goes to Backboard
                .forward(98).build(),
            traj3 = drive.trajectoryBuilder(traj2.end())
                    .strafeRight(240)
                    .build();

        while (!isStarted()){
            con1.update();

            if (con1.leftPressed){
                left = true;
            } else if(con1.rightPressed){
                left = false;
            }

            telemetry.addData("Left?", left);
            telemetry.update();
        }



        //waitForStart();

        if (!left) {
            drive.followTrajectory(traj1);
            //arm.setArmPosition(1);
            sleep(1000);
            while(opModeIsActive());
        } else{
            drive.followTrajectory(traj2);
            drive.followTrajectory(traj3);
        }
    }
}
