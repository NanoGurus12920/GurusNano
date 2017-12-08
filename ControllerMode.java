package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.LineNumberReader;

@TeleOp(name="ControllerMode", group="Linear Opmode")
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

        leftarm.setPosition(0.3); // the claw closes
        rightarm.setPosition(0.6); // the claw closes

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            /* For Both Wheel */
            double throttle = gamepad1.left_stick_y; // throttle equals the left joystick on the y-axis
            double turn     = gamepad1.left_stick_x; // turn equals the left joystick on the x-axis
            double leftspeed  = throttle - turn; // we take the throttle minue the turn to get the leftspeed
            double rightspeed = throttle + turn; // we take the throttle plus the turn to get the rightspeed
            leftMotor.setPower(leftspeed); // the left motor represents the leftspeed
            rightMotor.setPower(rightspeed); // the right motor represents the rightspeed


            //For Linear Slide
            linear.setPower(gamepad1.right_stick_x); // the linear slide goes up if the right joystick is pointing towards the x-axis
            linear.setPower(gamepad1.right_stick_y); // the linear slide down up if the right joystick is pointing towards the y-axis

            if (gamepad2.a) // if a is pressed
            {
                leftarm.setPosition(0.3); // the left claw is closed
                rightarm.setPosition(0.6); // the right claw is closed

            }

            if (gamepad2.b) // if b is pressed
            {
                leftarm.setPosition(0.1); // the left claw is open
                rightarm.setPosition(0.9); // the right claw is open
            }
        }
    }
}

