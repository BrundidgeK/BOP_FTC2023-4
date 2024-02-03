package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
public class RedBackstageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, -57.5, Math.toRadians(90)));

        //Moves to the center of the spike tile
        Trajectory spikeTraj = drive.trajectoryBuilder(new Pose2d(12, -57.5, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(12, -30.5, Math.toRadians(90))).build(),
                back = drive.trajectoryBuilder(spikeTraj.end()).lineTo(new Vector2d(12, -50)).build(),
                backboard1 = drive.trajectoryBuilder
                        (new Pose2d(spikeTraj.end().component1(), spikeTraj.end().component2(), 0))
                        .lineToConstantHeading(new Vector2d(51, -36)).build(),
               // backboard2 = drive.trajectoryBuilder(backboard1.end()).forward(24).build(),
                park1 = drive.trajectoryBuilder(backboard1.end()).strafeRight(24).build(),
                park2 = drive.trajectoryBuilder(park1.end()).forward(24).build();




        waitForStart();
        //drive.turn(Math.toRadians(-90));
        drive.followTrajectory(spikeTraj);
        System.out.println("Robot has upload code");
       // drive.followTrajectory(back);
        drive.turn(Math.toRadians(-90));
        System.out.println("Robot has upload code");
        drive.followTrajectory(backboard1);
        System.out.println("Robot has upload code");
        //drive.followTrajectory(backboard2);
        drive.followTrajectory(park1);
        drive.followTrajectory(park2);

        while (opModeIsActive()){
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().component1());
            telemetry.addData("y", drive.getPoseEstimate().component2());
            telemetry.addData("t", drive.getPoseEstimate().component3());
            telemetry.update();
        }
    }
}
