package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.DriveEngine.MecunamDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Bucket;
import org.firstinspires.ftc.teamcode.Subsystems.DroneLauncher;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.Subsystems.Suspension;
import org.firstinspires.ftc.teamcode.Util.Controller;

@TeleOp
public class BasicTele extends LinearOpMode {

    private MecunamDrive drive;
    private Arm arm;

    private Bucket bucket;
    private Lift lift;
    private Intake intake;
    //private ArmAndClaw armClaw;
    private DroneLauncher droneLauncher;


    @Override
    public void runOpMode() throws InterruptedException {
        Controller con1 = new Controller(gamepad1);
        Controller con2 = new Controller(gamepad2);
        //Inits mecanum drive
        //lift = new Lift(hardwareMap);

        drive = new MecunamDrive(hardwareMap);
        bucket = new Bucket(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap, this);
        intake = new Intake(hardwareMap);
        //armClaw = new ArmAndClaw(hardwareMap);
        Suspension sus = new Suspension(hardwareMap);
        droneLauncher = new DroneLauncher(hardwareMap);

        waitForStart();
        while (!isStopRequested()) {

            //Controls the lift
            if (con1.leftTriggerHeld) {
                lift.setPower(-con1.leftTrigger);
            } else if (con1.rightTriggerHeld) {
                lift.setPower(con1.rightTrigger);
            } else if (lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION){
                lift.setPosition(lift.getPosition(), 1
                );
            }
            //Use this to find the position lift needs to be above ground so px can get into place without lift being in way
            telemetry.addData("Lift Position", lift.getPosition());

            //Gets the gamepad inputs
            double left_x = con1.leftStick.x,
                    left_y = con1.leftStick.y,
                    right_x = con1.rightStick.x;

            telemetry.addData("left_x", con1.leftStick.x);
            telemetry.addData("left_y", con1.leftStick.y);
            telemetry.addData("right_x", con1.rightStick.x);

            drive.slowMode(con1.leftBumperHeld);

            //Gives the motors the power to move
            drive.moveWithPower(
                    -left_y - left_x + right_x,
                    left_y - left_x + right_x,
                    -left_y - left_x - right_x,
                    left_y - left_x - right_x
            );

            if (con1.aPressed) {
                droneLauncher.shoot();
            }


            if(con2.rightTriggerHeld){
                sus.setPower(con2.rightTrigger);
            } else if(con2.leftTriggerHeld){
                sus.setPower(-con2.leftTrigger);
            } else{
                sus.setPower(0);
            }

            if(con2.xPressed){
                sus.setHookPostion(sus.getHookPosition() == Suspension.CLOSE ? Suspension.OPEN : Suspension.CLOSE);
            }





            //Moves bucket based on current position
            if (con2.yPressed) {
                bucket.moveBucketIncrementally();
            } else {
                bucket.setServoPosition(Bucket.close);
            }

            if (con2.leftBumperHeld) {
                intake.intakeMotor.setPower(1);
            } else if (con2.rightBumperHeld) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if(con2.upHeld){
                //arm.moveBucketIncrementally(false);
                arm.setPower(1);
            } else if (con2.downHeld){
                //arm.moveBucketIncrementally(true);
                arm.setPower(-1);
            } else{
                arm.setPower(0);
            }
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Arm Target", arm.getTarget());
            telemetry.addData("Arm Heading", arm.getHeading());

            //Moves claw based on current position


            telemetry.addData("bucket", bucket.getServoPosition());
            //telemetry.addData("arm position", armClaw.getArmPosition());
            //telemetry.addData("claw position", armClaw.getClawPosition());
            telemetry.addData("Pixel", bucket.insideBucket());
            telemetry.addData("bucket", bucket.getServoPosition());

            telemetry.update();
            lift.update();
            intake.update();
            bucket.update();
            con1.update();
            con2.update();
            //sus.update();
        }
    }


    public void intakePosition(){
        lift.setPosition(0, 1);
        //  arm.setPosition(Arm.DOWN);

        arm.update();
    }
}