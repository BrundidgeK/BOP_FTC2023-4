package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
public class RedBackstageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Moves to the center of the spike tile
        Trajectory spikeTraj = drive.trajectoryBuilder(new Pose2d(12, -57.5, Math.toRadians(90))).forward(24).build(),
                spTraj = drive.trajectoryBuilder(spikeTraj.end()).strafeRight(24  ).build(),
                backUp = drive.trajectoryBuilder(spTraj.end()).back(24).build(),
                park = drive.trajectoryBuilder(backUp.end()).strafeRight(36).build();


        waitForStart();

        drive.followTrajectory(spikeTraj);
        drive.followTrajectory(spTraj);
        drive.followTrajectory(backUp);
        drive.followTrajectory(park);

    }
}
