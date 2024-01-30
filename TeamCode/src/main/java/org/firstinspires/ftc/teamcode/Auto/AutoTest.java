package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

@Autonomous
public class AutoTest extends LinearOpMode {

    public static int startX = 0, startY = 0, startH = 0;
    public static int endX = 0, endY = 0, endH = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory start = drive.trajectoryBuilder(new Pose2d(startX, startY), Math.toRadians(startH))
                .lineTo(new Vector2d(endX, endY)).build(),
                end = drive.trajectoryBuilder(start.end()).lineTo(
                        new Vector2d(startX, startY)).build();

        waitForStart();

        while(opModeIsActive()){
            drive.followTrajectory(start);
            sleep(5000);
            drive.followTrajectory(end);
            sleep(5000);
        }
    }
}
