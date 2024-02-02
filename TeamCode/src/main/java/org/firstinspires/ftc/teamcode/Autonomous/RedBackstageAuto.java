package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
        Trajectory spikeTraj = drive.trajectoryBuilder(new Pose2d(0,0,0)).lineTo(new Vector2d(27,0)).build(),
                door1 = drive.trajectoryBuilder(spikeTraj.end()).lineTo(new Vector2d(27, 24)).build(),
                door2 = drive.trajectoryBuilder(door1.end()).splineToSplineHeading(new Pose2d(48, -60), Math.toRadians(-90)).build(),
                back = drive.trajectoryBuilder(door2.end()).splineToConstantHeading(new Vector2d(27, -84), Math.toRadians(-90)).build();



        waitForStart();
        drive.followTrajectory(spikeTraj);
        drive.followTrajectory(door1);
        drive.followTrajectory(door2);
       // drive.followTrajectory(back);

        while (opModeIsActive()){
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().component1());
            telemetry.addData("y", drive.getPoseEstimate().component2());
            telemetry.addData("t", drive.getPoseEstimate().component3());
            telemetry.update();
        }
    }
}
