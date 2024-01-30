package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
public class colorSensorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSensor color = hardwareMap.get(ColorSensor.class, "color");
        RevBlinkinLedDriver led = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);

        waitForStart();

        while(opModeIsActive()){
            double r = color.red(),
                    g =  color.green(),
                    b =  color.blue();
            telemetry.addData("red", r);
            telemetry.addData("green", g);
            telemetry.addData("blue", b);
            int[] rgb = new int[]{(int) r, (int) g, (int) b};
            if (inrange(rgb, new int[]{220, 700, 375})) {
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } else if (inrange(rgb, new int[]{900, 1400, 511})){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (inrange(rgb, new int[]{200, 330, 475})){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else if (inrange(rgb, new int[]{785, 1625, 1750})){
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
            } else{
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
            telemetry.update();
        }
    }

    private boolean inrange(int[] rgb, int[] desired){
        int total = rgb[0] + rgb[1] + rgb[2],
                dTotal = desired[0] + desired[1] + desired[2];
        double percent = (double)total / (double) dTotal;

        return a(rgb[0], desired[0] * percent, 50) && a(rgb[1], desired[1] * percent, 50) && a(rgb[2], desired[2] * percent , 50) ;
    }

    boolean a (int a, double b, int c){
        return (a - c) <= b && (a + c) >= b;
    }


}
