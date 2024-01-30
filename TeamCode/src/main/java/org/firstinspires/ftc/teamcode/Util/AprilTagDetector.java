package org.firstinspires.ftc.teamcode.Util;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvPipeline;

public abstract class AprilTagDetector extends OpenCvPipeline {

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    public AprilTagDetector(HardwareMap hw){
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hw.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
    }

    public boolean tagFound(){
        return tagProcessor.getDetections().size() > 0;
    }
    public int tagFound(int id){
        for(int i = 0; i < tagProcessor.getDetections().size(); i++){
            if(tagProcessor.getDetections().get(i).id == id)
                return i;
        }
        return -1;
    }
    public int getId(){
        return tagProcessor.getDetections().get(0).id;
    }

    public double[] relativePosition(){
        if(tagFound()){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return new double[] {tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z};
        }
        return new double[0];
    }
    public double[] relativePosition(int id){
        int index = tagFound(id);
        if(index != -1){
            AprilTagDetection tag = tagProcessor.getDetections().get(index);
            return new double[] {tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z};
        }
        return new double[0];
    }

    public double[] relativeRotation(){
        if(tagFound()){
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            return new double[] {tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw};
        }
        return new double[0];
    }
    public double[] relativeRotation(int id){
        int index = tagFound(id);
        if(index != -1) {
            if (tagFound()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                return new double[]{tag.ftcPose.roll, tag.ftcPose.pitch, tag.ftcPose.yaw};
            }
        }
        return new double[0];
    }
}
