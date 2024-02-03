package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

@Autonomous
public class RedBackstageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(12, -57.5, Math.toRadians(90)));

        //Moves to the center of the spike tile
        Trajectory spike = drive.trajectoryBuilder(new Pose2d(12, -57.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(12, -30, Math.toRadians(90))).build(),
                backboard = drive.trajectoryBuilder(spike.end())
                        .lineToConstantHeading(new Vector2d(48, -30)).build();


        waitForStart();

        drive.followTrajectory(spike);
        drive.followTrajectory(backboard);
        drive.turn(Math.toRadians(-90));

    }
}
