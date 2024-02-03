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
        drive.setPoseEstimate(new Pose2d(-36, -61, Math.toRadians(90)));

        Trajectory startTraj = drive.trajectoryBuilder(
                new Pose2d(-36,-61,Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36,-34, Math.toRadians(0))).build(),
                strafe = drive.trajectoryBuilder(startTraj.end()).strafeLeft(24).build();

        waitForStart();

        drive.followTrajectory(startTraj);
        drive.followTrajectory(strafe);
    }
}
