package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneLauncher{
    private Servo droneLauncher;
    public static final double D_SHOOT = 0, D_CLOSE = 1;// Positions for drone launcher servo

    public DroneLauncher(HardwareMap hwMap) {
        droneLauncher = hwMap.get(Servo.class, "servoDrone");
    }

    // Method to open the drone launcher
    public void shoot() {
        droneLauncher.setPosition(droneLauncher.getPosition() == D_SHOOT ? D_CLOSE : D_SHOOT);
    }

    // Method to close the drone launcher
    // public void close() {
    //    droneLauncher.setPosition(D_CLOSE);
    //}
}