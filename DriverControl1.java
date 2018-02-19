package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "DriverControl1", group = "Linear Opmode")
// @Autonomous(...) is the other common choice
public class DriverControl1 extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftEscalator;
    private DcMotor rightEscalator;
    private DcMotor leftBottom;
    private DcMotor rightBottom;
    private Servo leftArm = null;
    private Servo booster =null;

    /**
     * This is the code for moving the bot using only one joystick, where the escalator has four motors, the wheels are being powered using four motors, our jewel arm is coming up using one servo, and our high torque servo is powering our top part of our escalator, bring us enough momentum to reach the fourth row.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftEscalator = hardwareMap.dcMotor.get("leftEscalator");
        rightEscalator = hardwareMap.dcMotor.get("rightEscalator");
        leftBottom = hardwareMap.dcMotor.get("leftBottom");
        rightBottom = hardwareMap.dcMotor.get("rightBottom");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        booster = hardwareMap.get(Servo.class, "booster");

        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftArm.setPosition(0.4);
            booster.setPosition(0.6);

            double x = gamepad1.left_stick_x;
            double y     = gamepad1.left_stick_y;
            double leftspeed  = x - y ;
            double rightspeed = x + y ;

            leftMotor.setPower(leftspeed);
            rightMotor.setPower(rightspeed);

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
