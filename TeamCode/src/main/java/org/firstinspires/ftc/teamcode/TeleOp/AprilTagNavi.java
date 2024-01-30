package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.AprilTagDetector;
import org.opencv.core.Mat;

@TeleOp
public class AprilTagNavi extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagDetector tagDetector = new AprilTagDetector(hardwareMap) {
            @Override
            public Mat processFrame(Mat input) {
                return null;
            }
        };

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Tag ID", tagDetector.getId());

            double[] relPos = tagDetector.relativePosition(),
                    relRot = tagDetector.relativeRotation();


            telemetry.addData("x", relPos[0]);
            telemetry.addData("y", relPos[1]);
            telemetry.addData("z", relPos[2]);
            telemetry.addLine();

            telemetry.addData("x", relRot[0]);
            telemetry.addData("y", relRot[1]);
            telemetry.addData("z", relRot[2]);

            telemetry.update();
        }
    }
}
