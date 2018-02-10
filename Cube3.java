package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Cube3", group = "Linear Opmode")  // @Autonomous(...) is the other common choice
public class Cube3 extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftEscalator;
    private DcMotor rightEscalator;
    private DcMotor leftBottom;
    private DcMotor rightBottom;
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
            leftBottom = hardwareMap.dcMotor.get("leftBottom");
            rightBottom = hardwareMap.dcMotor.get("rightBottom");
            largeJewelArm = hardwareMap.get(Servo.class, "leftarm");

            waitForStart();
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                largeJewelArm.setPosition(0.4);
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();

                double x = gamepad1.left_stick_x;
                double y     = gamepad1.left_stick_y;
                double leftspeed  = x - y ;
                double rightspeed = x + y ;

                leftMotor.setPower(leftspeed);
                rightMotor.setPower(rightspeed);

                boolean isTopEscalatorRunning=false;
                if (gamepad2.right_bumper) {
                    if (isTopEscalatorRunning == true){        //Stop Top Escalator
                        leftEscalator.setPower(0);
                        rightEscalator.setPower(0);
                        isTopEscalatorRunning=true;
                    }else{                                  //Start Top Escalator
                        leftEscalator.setPower(0.7);
                        rightEscalator.setPower(-0.7);
                        isTopEscalatorRunning=false;
                    }
                }

                boolean isBottomEscalatorRunning = false;
                if (gamepad2.left_bumper) {
                    if (isBottomEscalatorRunning == true){       //Stop Bottom Escalator
                        leftBottom.setPower(0);
                        rightBottom.setPower(0);
                        isBottomEscalatorRunning=true;
                    }else{                                      //Start Bottom Escalator
                        leftBottom.setPower(0.7);
                        rightBottom.setPower(-0.7);
                        isBottomEscalatorRunning=false;
                }

            }
        }
    }
    }