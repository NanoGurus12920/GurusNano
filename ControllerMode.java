package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.LineNumberReader;

@TeleOp(name="TeleOp", group="Linear Opmode")
public class ControllerMode extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor linear;
    private Servo leftarm;
    private Servo rightarm;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        linear = hardwareMap.dcMotor.get("linear");
        leftarm = hardwareMap.servo.get("leftarm");
        rightarm = hardwareMap.servo.get("rightarm");

        leftarm.setPosition(0.3);
        rightarm.setPosition(0.6);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            /* For Both Wheel */
                double throttle = gamepad1.left_stick_y;
                double turn     = gamepad1.left_stick_x;
                double leftspeed  = throttle - turn;
                double rightspeed = throttle + turn;
                leftMotor.setPower(leftspeed);
                rightMotor.setPower(rightspeed);

            // Try the top of bottom
            /*leftMotor.setPower(gamepad1.left_stick_x);
            rightMotor.setMode(gamepad1.left_stick_y);*/


            //For Linear Slide
            linear.setPower(gamepad1.right_stick_x);
            linear.setPower(gamepad1.right_stick_y);

            if (gamepad2.a)
            {
                leftarm.setPosition(0.3);
                rightarm.setPosition(0.6);

            }

            if (gamepad2.b)
            {
                leftarm.setPosition(0.1);
                rightarm.setPosition(0.9);
            }
        }
    }
}

