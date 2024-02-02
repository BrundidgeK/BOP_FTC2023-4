package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
public class RedFrontStageAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory startTraj = drive.trajectoryBuilder(new Pose2d(0,0)).lineToConstantHeading(new Vector2d(12, 27)).build(),
                strage = drive.trajectoryBuilder(startTraj.end()).strafeLeft(12).build(),
                door2 = drive.trajectoryBuilder(strage.end()).splineToSplineHeading(new Pose2d(48, -60), Math.toRadians(-90)).build();

        waitForStart();

        drive.followTrajectory(startTraj);
        drive.followTrajectory(strage);
        drive.followTrajectory(door2);
    }
}
