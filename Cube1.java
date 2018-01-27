package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Cube1", group = "Linear Opmode")  // @Autonomous(...) is the other common choice
public class Cube1 extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftEscalator;
    private DcMotor rightEscalator;
    private Servo largeJewelArm   = null;

    /**
     * This is the code for moving the bot using only one joystick
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftEscalator = hardwareMap.dcMotor.get("leftEscalator");
        rightEscalator = hardwareMap.dcMotor.get("rightEscalator");
        largeJewelArm = hardwareMap.get(Servo.class, "leftarm");

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            largeJewelArm.setPosition(0.3);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            double x = gamepad1.left_stick_x;
            double y     = gamepad1.left_stick_y;
            double leftspeed  = x - y ;
            double rightspeed = x + y ;

            leftMotor.setPower(leftspeed);
            rightMotor.setPower(rightspeed);

            // Start Escalator
            if (gamepad2.right_bumper) {
                leftEscalator.setPower(0.7);
                rightEscalator.setPower(-0.7);
            }

            // Stop Escalator
            if (gamepad2.left_bumper) {
                leftEscalator.setPower(0);
                rightEscalator.setPower(0);
            }

        }
    }
}