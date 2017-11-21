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

            //For Both Wheel
            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(-gamepad1.right_stick_x);

            //For Linear Slide
                linear.setPower(gamepad1.right_stick_x);
                linear.setPower(gamepad1.right_stick_y);
            }
        }
    }


