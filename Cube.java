package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.LineNumberReader;

@TeleOp(name="Cube", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class Cube extends LinearOpMode {
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

        //leftMotor.setDirection(DcMotor.Direction.REVERSE);


        leftarm.setPosition(0.3);
        rightarm.setPosition(0.6);

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // float left_power = -gamepad1.left_stick_y;
            // float right_power = -gamepad1.left_stick_y;

            leftMotor.setPower(gamepad1.left_stick_y);
            rightMotor.setPower(-gamepad1.right_stick_x);

            if (gamepad1.a)
            {
                leftarm.setPosition(0.3);
                rightarm.setPosition(0.6);

            }

            if (gamepad1.b)
            {
                leftarm.setPosition(0.1);
                rightarm.setPosition(0.9);
            }

            if(gamepad1.right_bumper)
            {
                linear.setPower(0.7);
            }
            else
            {
                linear.setPower(0);
            }
            // Linear Slides
            if(gamepad1.left_bumper)
            {
                linear.setPower(-0.7);
            }
            else
            {
                linear.setPower(0);
            }
        }
    }
}
