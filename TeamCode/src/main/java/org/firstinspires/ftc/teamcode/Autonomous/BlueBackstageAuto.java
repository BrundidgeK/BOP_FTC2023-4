package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Deprecated
@Autonomous
public class BlueBackstageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory startTraj = drive.trajectoryBuilder(new Pose2d(12, 57.5), Math.toRadians(-90)) //Goes to Backboard
                .lineToSplineHeading(new Pose2d(12, 32.5, Math.toRadians(180)))
                .build(),
                spikeTraj = drive.trajectoryBuilder(startTraj.end())
                        .lineToSplineHeading(new Pose2d(48, 32.5, 0)).build(),
                /*pixelStackTraj = drive.trajectoryBuilder(startTraj.end())
                        .lineToConstantHeading(new Vector2d(-36, -35.5))
                        .lineToSplineHeading(new Pose2d(-60, -35.5, Math.toRadians(180)))
                        .build(),
                backboardTraj = drive.trajectoryBuilder(pixelStackTraj.end())
                        .lineToConstantHeading(new Vector2d(24, -35.5))
                        .lineToSplineHeading(new Pose2d(48, -35.5, 0))
                        .build(),*/
                park1 = drive.trajectoryBuilder(spikeTraj.end())
                        .strafeLeft(24)
                        .build(),
                park2 = drive.trajectoryBuilder(park1.end())
                        .forward(12)
                        .build();


        waitForStart();

        drive.followTrajectory(startTraj); //Goes to backboard to drop yellow
        drive.followTrajectory(spikeTraj);
        //Parks
        drive.followTrajectory(park1);
        drive.followTrajectory(park2);
    }
}
