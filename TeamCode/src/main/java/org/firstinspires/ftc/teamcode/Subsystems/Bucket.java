package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;

public class Bucket {
    private Servo bucket;
    private ColorSensor color;
    public static final double open = 0.5, close = 0.35;
    public static final double INCREMENT = 0.1; // increment value
    public static final double DELAY_SECONDS = 0.2;// delay in seconds

    public Bucket(HardwareMap hwMap) {
        bucket = hwMap.get(Servo.class, "bucket");
        color = hwMap.get(ColorSensor.class, "color");
    }

    public void init(HardwareMap hwMap) {
        bucket = hwMap.get(Servo.class, "bucket");
    }

    public boolean setServoPosition(double position) {
        bucket.setPosition(position);
        return true;
    }

    public double getServoPosition() {
        return bucket.getPosition();
    }

    public void moveBucketIncrementally() {
        double currentPosition = getServoPosition();
        double targetPosition = open; // Set the target position, e.g., fully open

        // Incrementally move the servo towards the target position
        while (currentPosition < targetPosition) {
            currentPosition += INCREMENT;
            setServoPosition(currentPosition);
            // Add a delay to create a natural movement
            sleepSeconds(DELAY_SECONDS);
        }
    }

    private void sleepSeconds(double seconds) {
        try {
            Thread.sleep((long) (seconds * 1000));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public String insideBucket(){
        int[] rgb = new int[]{color.red(), color.green(), color.blue()},
                sorted = rgb.clone();

        Arrays.sort(sorted);
        // green - g, b, r
        // yellow - g, r, b
        // purple - b, g, r
        // white - b, g, r

        if (sorted[0] == rgb[1] && sorted[1] == rgb[0]) { //If green is high
            return "yellow";
        } else if (sorted[0] == rgb[1] && sorted[1] == rgb[1]){
            return "green";
        }else if (sorted[0] == rgb[2] && sorted[1] == rgb[1]){
            return "white";
        } else{
            return "black";
        }
    }

    public void update() {
        // Add any additional update logic here if needed
    }
}
