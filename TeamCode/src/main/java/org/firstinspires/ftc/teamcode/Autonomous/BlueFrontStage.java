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
public class BlueFrontStage extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Arm arm = new Arm(hardwareMap, this);
        Bucket bucket = new Bucket(hardwareMap);
        Lift lift = new Lift(hardwareMap);
        Camera cam = new Camera(hardwareMap);

        String spikePos = "right";

        Pose2d startPose = new Pose2d(-36, 61.25, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        //Moves to the center of the spike tile
        Trajectory spikeTraj = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-36, -36))
                .build();

        //If the egg is in middle
        TrajectorySequence centerPlacePurple = drive.trajectorySequenceBuilder(spikeTraj.end())
                .lineTo(new Vector2d(-36, 25))
                .lineTo(new Vector2d(-36, 42))
                .build();
        TrajectorySequence centerStageDoor = drive.trajectorySequenceBuilder(centerPlacePurple.end()
                        .plus(new Pose2d(0,0,Math.toRadians(-90))))
                .lineTo(new Vector2d(-48, 42))
                .lineTo(new Vector2d(-48, 12))
                .lineTo(new Vector2d(48, 12))
                .build();
        Trajectory centerBackBoard = drive.trajectoryBuilder(centerPlacePurple.end())
                .lineTo(new Vector2d(48, 36))
                .lineTo(new Vector2d(52, 36))
                .build();
        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(centerBackBoard.end())
                .lineTo(new Vector2d(48, 36))
                .lineTo(new Vector2d(48, 60))
                .build();

        //If the egg is on the left
        TrajectorySequence leftPlacePurple = drive.trajectorySequenceBuilder
                        (spikeTraj.end().plus(new Pose2d(0,0, Math.toRadians(90))))
                .lineTo(new Vector2d(-25, 36))
                .lineTo(new Vector2d(-42, 36))
                .lineTo(new Vector2d(-42, 12))
                .lineTo(new Vector2d(48, 12))
                .build();
        TrajectorySequence leftBackBoard = drive.trajectorySequenceBuilder(leftPlacePurple.end())
                .lineTo(new Vector2d(48, 40))
                .lineTo(new Vector2d(52, 40))
                .build();
        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(leftBackBoard.end())
                .lineTo(new Vector2d(48, 40))
                .lineTo(new Vector2d(48, 60))
                .build();

        TrajectorySequence rightPlacePurple = drive.trajectorySequenceBuilder(
                        (spikeTraj.end().plus(new Pose2d(0,0, Math.toRadians(-90)))))
                .lineTo(new Vector2d(-45, 36))
                .lineTo(new Vector2d(-35, 36))
                .lineTo(new Vector2d(-35, 12))
                .build();
        Trajectory rightStageDoor = drive.trajectoryBuilder(rightPlacePurple.end()
                .plus(new Pose2d(0,0,Math.toRadians(180))))
                .lineTo(new Vector2d(48, 12))
                .build();
        TrajectorySequence rightBackBoard = drive.trajectorySequenceBuilder(rightStageDoor.end())
                .lineTo(new Vector2d(48, 28))
                .lineTo(new Vector2d(52, 28))
                .build();
        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(rightBackBoard.end())
                .lineTo(new Vector2d(48, 28))
                .lineTo(new Vector2d(48, 60))
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
                drive.turn(Math.toRadians(-90));
                drive.followTrajectorySequence(rightPlacePurple);
                drive.turn(Math.toRadians(180));
                drive.followTrajectory(rightStageDoor);

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
                drive.turn(Math.toRadians(90));
                drive.followTrajectorySequence(centerStageDoor);
                lift.setPosition(1250, 1);
                while(lift.isBusy() && opModeIsActive());
                drive.followTrajectory(centerBackBoard);
                bucket.moveBucketIncrementally();
                sleep(2500);
                drive.followTrajectorySequence(centerPark);
                bucket.setServoPosition(Bucket.close);

                break;

            case "left":
                drive.turn(Math.toRadians(90));
                drive.followTrajectorySequence(leftPlacePurple);
                lift.setPosition(1250, 1);
                drive.followTrajectorySequence(leftBackBoard);
                bucket.moveBucketIncrementally();
                sleep(2500);
                drive.followTrajectorySequence(leftPark);
                bucket.setServoPosition(Bucket.close);
                break;

        }
        while (opModeIsActive()){
            drive.updatePoseEstimate();

            telemetry.addData("x", drive.getPoseEstimate().component1());
            telemetry.addData("y", drive.getPoseEstimate().component2());
            telemetry.addData("t", drive.getPoseEstimate().component3());
            telemetry.update();
        }
    }
}

