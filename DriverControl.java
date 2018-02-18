package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverControl", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
public class DriverControl extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftEscalator;
    private DcMotor rightEscalator;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBottom;
    private DcMotor rightBottom;
    private Servo largeJewelArm = null;
    private Servo booster =null;

    /**
     * This is the code for moving the bot using only one joystick
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.dcMotor.get("rightMotor");
        rightFront = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftEscalator = hardwareMap.dcMotor.get("leftEscalator");
        rightEscalator = hardwareMap.dcMotor.get("rightEscalator");
        leftBottom = hardwareMap.dcMotor.get("leftBottom");
        rightBottom = hardwareMap.dcMotor.get("rightBottom");
        largeJewelArm = hardwareMap.get(Servo.class, "leftarm");
        booster = hardwareMap.get(Servo.class, "booster");

        waitForStart();
        // run until the end of the match (driver presses STOP)
        booster.setPosition(0.5);
        while (opModeIsActive()) {
            largeJewelArm.setPosition(0.4);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

                /*double x = gamepad1.left_stick_x;
                double y     = gamepad1.left_stick_y;
                double leftspeed  = x - y ;
                double rightspeed = x + y ;

                leftMotor.setPower(leftspeed);
                rightMotor.setPower(rightspeed);*/

            // Movement and Strafing
            float LFspeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
            float LBspeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
            float RFspeed= gamepad1.right_stick_y + gamepad1.left_stick_x;
            float RBspeed = gamepad1.right_stick_y - gamepad1.left_stick_x;

            LFspeed = Range.clip(LFspeed, -1, 1);
            LBspeed = Range.clip(LBspeed, -1, 1);
            RFspeed = Range.clip(RFspeed, -1, 1);
            RBspeed = Range.clip(RBspeed, -1, 1);

            leftFront.setPower(LFspeed);
            leftMotor.setPower(LBspeed);
            rightFront.setPower(RFspeed);
            rightMotor.setPower(RBspeed);


            if (gamepad2.left_bumper) {                         //Start Top Escalator
                leftEscalator.setPower(1);
                rightEscalator.setPower(-1);
            } else {                                  //Stop Top Escalator
                leftEscalator.setPower(0);
                rightEscalator.setPower(0);
            }
            if (gamepad2.right_bumper) {                    //Start Bottom Escalator
                leftBottom.setPower(1);
                rightBottom.setPower(-1);
            } else {                                      //Start Bottom Escalator
                leftBottom.setPower(0);
                rightBottom.setPower(0);
            }

        }
    }
}