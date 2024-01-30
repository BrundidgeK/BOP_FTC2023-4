package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;

import java.util.List;

@TeleOp
public class teamPropDetection extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Camera cam = new Camera(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            List<Recognition> recognitionList = cam.getRecognitions();
            for (Recognition recognition : recognitionList) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

                telemetry.update();
            }
            cam.stopStream();
        }


    }
}
