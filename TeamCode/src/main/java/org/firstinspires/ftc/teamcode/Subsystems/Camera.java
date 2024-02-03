package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    private VisionPortal visionPortal;
    private TfodProcessor tfodProcessor;

    private double confidenceMin = 80;
    private double midPos, rightPos;

    public Camera(HardwareMap hw){
        tfodProcessor = new TfodProcessor.Builder()
                .setModelFileName("Egg.tflite")
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hw.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 480));
        //builder.enableCameraMonitoring(true);

        builder.addProcessor(tfodProcessor);
        visionPortal = builder.build();

        visionPortal.setProcessorEnabled(tfodProcessor, true);
    }

    public List<Recognition> getRecognitions(){
        return tfodProcessor.getRecognitions();
    }
    public List<Recognition> getRecognitions(String label){
        List<Recognition> recognitions = new ArrayList<Recognition>();
        
        for (Recognition r : tfodProcessor.getRecognitions()){
            if(r.getLabel().equals(label) && r.getConfidence() >= confidenceMin)
                recognitions.add(r);
        }
        
        return recognitions;
    }

    public int getPosition(String label){
        Recognition egg = getRecognitions(label).get(0);
        float xPosition = (egg.getLeft() + egg.getRight()) / 2.0f;

        if(xPosition < midPos){ //left
            return 0;
        } else if (xPosition >= midPos && xPosition < rightPos){ //mid
            return 1;
        } else{ //right
            return 2;
        }
    }
    
    public void stopStream(){
        visionPortal.stopStreaming();
    }

    public void startStream(){
        visionPortal.resumeStreaming();
        int x = getPosition("red");
        x = getPosition("red");
        x = getPosition("red");
        x = getPosition("red");
    }
}