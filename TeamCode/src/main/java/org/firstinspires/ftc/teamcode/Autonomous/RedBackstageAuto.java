package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Bucket;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

import java.util.List;

@Autonomous
public class RedBackstageAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap, this);
        Bucket bucket = new Bucket(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Camera cam = new Camera(hardwareMap);

        String spikePos = "right";

        Pose2d startPose = new Pose2d(12, -61.25, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        //Moves to the center of the spike tile
        Trajectory spikeTraj = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(12, -36, Math.toRadians(90)))
                .build();

        //If the egg is in middle
        TrajectorySequence centerPlacePurple = drive.trajectorySequenceBuilder(spikeTraj.end())
                .lineTo(new Vector2d(12, -25))
                .lineTo(new Vector2d(12, -37))
                .build();
        Trajectory centerBackBoard = drive.trajectoryBuilder(centerPlacePurple.end().plus(new Pose2d(0,0, Math.toRadians(-90))))
                .lineTo(new Vector2d(52, -36))
                .build();
        Trajectory centerPark = drive.trajectoryBuilder(centerBackBoard.end())
                .lineTo(new Vector2d(48, -60))
                .build();

        //If the egg is on the left
        TrajectorySequence leftPlacePurple = drive.trajectorySequenceBuilder
                        (spikeTraj.end().plus(new Pose2d(0,0, Math.toRadians(-90))))
                .lineTo(new Vector2d(24, -37))
                .lineTo(new Vector2d(12, -36))
                .build();
        Trajectory leftStrafe = drive.trajectoryBuilder(leftPlacePurple.end())
                .lineTo(new Vector2d(12, -48))
                .build();
        TrajectorySequence leftBackBoard = drive.trajectorySequenceBuilder(leftStrafe.end())
                .lineTo(new Vector2d(48, -48))
                .lineTo(new Vector2d(48, -28))
                .lineTo(new Vector2d(52, -28))
                .build();
        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(leftBackBoard.end())
                .lineTo(new Vector2d(48, -40))
                .lineTo(new Vector2d(48, -60))
                .build();

        Trajectory rightPlacePurple = drive.trajectoryBuilder
                        (spikeTraj.end().plus(new Pose2d(0,0, Math.toRadians(90))))
                .lineTo(new Vector2d(1, -36))
                .build();
        Trajectory rightBack = drive.trajectoryBuilder(rightPlacePurple.end())
                .lineTo(new Vector2d(12, -36))
                .build();
        TrajectorySequence rightBackBoard = drive.trajectorySequenceBuilder(rightBack.end().plus(new Pose2d(0,0,Math.toRadians(180))))
                .lineTo(new Vector2d(48, -36))
                .lineTo(new Vector2d(48, -40))
                .lineTo(new Vector2d(52, -40))
                .build();
        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(rightBackBoard.end())
                .lineTo(new Vector2d(48, -31))
                .lineTo(new Vector2d(48, -60))
                .build();

        Trajectory backboard1 = drive.trajectoryBuilder(spikeTraj.end().plus(new Pose2d(0, 0, Math.toRadians(-90))))
                .lineToLinearHeading(new Pose2d(51, -36, Math.toRadians(0)))
                .build();

        // backboard2 = drive.trajectoryBuilder(backboard1.end()).forward(24).build(),
        Trajectory park1 = drive.trajectoryBuilder(backboard1.end()).strafeLeft(24).build();

//        Trajectory park2 = drive.trajectoryBuilder(park1.end()).forward(24).build();

        while(!isStarted() && !isStopRequested()){
            spikePos = cam.getPosition();
            telemetry.addData("Position spike", cam.getPosition());
            telemetry.update();
        }

        cam.stopStream();

        //Moves to center of spike tile
        arm.setPosition(Arm.SCORING_POS, -1);
        while(opModeIsActive() && arm.isBusy()){
            arm.update();
        }
        drive.followTrajectory(spikeTraj);

        switch (spikePos) {
            case "right":
                drive.turn(Math.toRadians(90));
                drive.followTrajectory(rightPlacePurple);
                drive.followTrajectory(rightBack);
                drive.turn(Math.toRadians(180));

                lift.setPosition(1250, 1);
                while(opModeIsActive() && lift.isBusy());
                drive.followTrajectorySequence(rightBackBoard);
                bucket.moveBucketIncrementally();
                sleep(2500);
                drive.followTrajectorySequence(rightPark);
                bucket.setServoPosition(Bucket.close);
                break;

            case "middle":
                drive.followTrajectorySequence(centerPlacePurple);
                drive.turn(Math.toRadians(-90));
                lift.setPosition(1250, 1);
                while(lift.isBusy() && opModeIsActive());
                drive.followTrajectory(centerBackBoard);
                bucket.moveBucketIncrementally();
                sleep(2500);
                drive.followTrajectory(centerPark);
                bucket.setServoPosition(Bucket.close);

                break;

            case "left":
                drive.turn(Math.toRadians(-90));
                drive.followTrajectorySequence(leftPlacePurple);
                drive.followTrajectory(leftStrafe);
                lift.setPosition(1250, 1);
                drive.followTrajectorySequence(leftBackBoard);
                bucket.moveBucketIncrementally();
                sleep(2500);
                drive.followTrajectorySequence(leftPark);
                bucket.setServoPosition(Bucket.close);
                break;

        }


        /*
        //drive.turn(Math.toRadians(-90));
        arm.setPosition(Arm.SCORING_POS, -1);
        while(opModeIsActive() && arm.isBusy()){
            arm.update();
        }
        drive.followTrajectory(spikeTraj);
        lift.setPosition(1600, .5);
        while(opModeIsActive() && lift.isBusy());
        bucket.moveBucketIncrementally();
        sleep(2000);
        bucket.setServoPosition(Bucket.close);

        drive.followTrajectory(back);
        drive.turn(Math.toRadians(90));
        //lift.setPosition(1600, 1);
        //while(opModeIsActive() && lift.isBusy());
        drive.followTrajectory(backboard1);
        bucket.moveBucketIncrementally();
        sleep(2000);
        bucket.setServoPosition(Bucket.close);
        bucket.moveBucketIncrementally();
        sleep(2000);
        bucket.setServoPosition(Bucket.close);
//        //drive.followTrajectory(backboard2);
        drive.followTrajectory(park1);
//        drive.followTrajectory(park2);


         */
        while (opModeIsActive()){
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().component1());
            telemetry.addData("y", drive.getPoseEstimate().component2());
            telemetry.addData("t", drive.getPoseEstimate().component3());
            telemetry.update();
        }
    }
}

