package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
public class BlueFrontStage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 57.5, Math.toRadians(-90)));


        Trajectory spike = drive.trajectoryBuilder(new Pose2d(-36, 57.5, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(-36, 30)).build(),
                truss1 = drive.trajectoryBuilder(spike.end()).back(24).build(),
                truss2 = drive.trajectoryBuilder(truss1.end()).lineToSplineHeading
                        (new Pose2d(48, -54, Math.toRadians(-90))).build(),
                backBoard = drive.trajectoryBuilder(truss2.end().plus(
                                new Pose2d(0,0, Math.toRadians(90)))).strafeRight(18)
                        .build();


        waitForStart();
        drive.followTrajectory(spike);
        drive.followTrajectory(truss1);
        drive.followTrajectory(truss2);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(backBoard);
        drive.followTrajectory(spike);
    }
}
