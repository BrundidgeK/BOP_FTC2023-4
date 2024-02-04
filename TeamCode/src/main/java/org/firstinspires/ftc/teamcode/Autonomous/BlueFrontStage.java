package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Bucket;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Autonomous
public class BlueFrontStage extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Camera cam = new Camera(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 61.5, Math.toRadians(-90)));
        Arm arm = new Arm(hardwareMap, this);
        Bucket bucket = new Bucket(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        String spikePos = "right";

        Trajectory spike = drive.trajectoryBuilder(new Pose2d(-36, 61.5, Math.toRadians(-90)))
                .lineToConstantHeading(new Vector2d(-36, 35 )).build(),
                truss1 = drive.trajectoryBuilder(spike.end())
                        .lineToConstantHeading(new Vector2d(-36, 36)).build(),
                truss2 = drive.trajectoryBuilder(truss1.end()).strafeRight(24).build(),
                centerstage1 = drive.trajectoryBuilder(truss2.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                                .strafeRight(24).build(),
                centerstage2 = drive.trajectoryBuilder(centerstage1.end())
                        .lineToConstantHeading(new Vector2d(48, 12)).build(),
                backBoard1 = drive.trajectoryBuilder(centerstage2.end())
                        .lineToConstantHeading(new Vector2d(48, 36))
                        .build(),
                backBoard2 = drive.trajectoryBuilder(backBoard1.end()).lineToConstantHeading
                        (new Vector2d(53,36)).build(),
                park1 = drive.trajectoryBuilder(backBoard2.end())
                        .lineToConstantHeading(new Vector2d(48, 36)).build(),
                park2 = drive.trajectoryBuilder(park1.end())
                        .lineToConstantHeading(new Vector2d(48, 60)).build();


        while(!isStarted() && !isStopRequested()){
            spikePos = cam.getPosition();
            telemetry.addData("Position spike", cam.getPosition());
            telemetry.update();
        }

        cam.stopStream();

        arm.setPosition(Arm.SCORING_POS, -1);
        while(opModeIsActive() && arm.isBusy()){
            arm.update();
        }
        drive.followTrajectory(spike);
        lift.setPosition(700, .5);
        while(opModeIsActive() && lift.isBusy());
        sleep(1000);
        bucket.moveBucketIncrementally();
        sleep(2000);
        bucket.setServoPosition(Bucket.close);
        drive.followTrajectory(truss1);
        drive.followTrajectory(truss2);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(centerstage1);
        lift.setPosition(0, .5);
        while(opModeIsActive() && lift.isBusy());
        drive.followTrajectory(centerstage2);
        drive.followTrajectory(backBoard1);
        lift.setPosition(1600, .5);
        while(opModeIsActive() && lift.isBusy());
        drive.followTrajectory(backBoard2);
        bucket.moveBucketIncrementally();
        sleep(2000);
        bucket.setServoPosition(Bucket.close);
        bucket.moveBucketIncrementally();
        sleep(2000);
        bucket.setServoPosition(Bucket.close);
        drive.followTrajectory(park1);
        drive.followTrajectory(park2);
    }
}
