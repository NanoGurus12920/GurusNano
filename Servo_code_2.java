package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Locale;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

@Autonomous(name="Servo_code_2", group="test")
//@Disabled
public class Servo_code_2 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    Servo Servo2;

    ColorSensor SensorColor2;
    //DistanceSensor sensorDistance2;

    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorLeft2 = null;
    DcMotor motorRight2 = null;
    DcMotor motor5 = null;
    
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3);
    static final double     DRIVE_SPEED             = 0.5;
  


    @Override
    public void runOpMode() {

    Servo2 = hardwareMap.servo.get("Servo2");
      
    motorLeft  = hardwareMap.dcMotor.get("motorLeft");
    motorRight = hardwareMap.dcMotor.get("MotorRight");
    motorLeft2  = hardwareMap.dcMotor.get("motorLeft2");
    motorRight2 = hardwareMap.dcMotor.get("MotorRight2");
    motor5 = hardwareMap.dcMotor.get("motor_5");
    
    SensorColor2 = hardwareMap.get(ColorSensor.class, "sensor_color_distance_2");
    //sensorDistance2 = hardwareMap.get(DistanceSensor.class, "sensor_color_distance_2");
    
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorLeft2.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.FORWARD);
        motor5.setDirection(DcMotor.Direction.REVERSE);
        
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        idle();

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

 
        telemetry.addData("Path0",  "Starting at %7d :%7d",
        
                motorLeft.getCurrentPosition(),
                motorRight.getCurrentPosition());
                motorLeft2.getCurrentPosition();
                motorRight2.getCurrentPosition();
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
         
         
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        Servo2.setPosition(0.62);

        runtime.reset();
        
            telemetry.addData("Red  ", SensorColor2.red());
            telemetry.addData("Green", SensorColor2.green());
            telemetry.addData("Blue ", SensorColor2.blue());
			telemetry.update();

        if(SensorColor2.blue() > 20 ){
        encoderDrive(DRIVE_SPEED, -2.0, -2.0, 1.0);
			            telemetry.addData("Red  ", SensorColor2.red());
            telemetry.addData("Green", SensorColor2.green());
            telemetry.addData("Blue ", SensorColor2.blue());
			telemetry.update();
		}
        else if(SensorColor2.blue() < 20 ){
        encoderDrive(DRIVE_SPEED, 4.5, 4.5, 5.0);
			            telemetry.addData("Red  ", SensorColor2.red());
            telemetry.addData("Green", SensorColor2.green());
            telemetry.addData("Blue ", SensorColor2.blue());
			telemetry.update();
        }
        
		
		
		telemetry.addData("Red  ", SensorColor2.red());
            telemetry.addData("Green", SensorColor2.green());
            telemetry.addData("Blue ", SensorColor2.blue());
			telemetry.update();
		
		
		
        sleep(1000);
        
		
		
		telemetry.addData("Red  ", SensorColor2.red());
            telemetry.addData("Green", SensorColor2.green());
            telemetry.addData("Blue ", SensorColor2.blue());
			telemetry.update();
        Servo2.setPosition(-0.6);
   //  sleep(250);   // optional pause after each move
		
		
		
		
		
		
		
		 public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            
            motorLeft.setTargetPosition(newLeftTarget);
            motorRight.setTargetPosition(newRightTarget);
            motorLeft2.setTargetPosition(newLeftTarget);
            motorRight2.setTargetPosition(newRightTarget);

            motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRight2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            runtime.reset();
            motorLeft.setPower(Math.abs(speed));
            motorRight.setPower(Math.abs(speed));
            motorLeft2.setPower(Math.abs(speed));
            motorRight2.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motorLeft.isBusy() && motorRight.isBusy())) {

              
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        motorLeft.getCurrentPosition(),
                        motorRight.getCurrentPosition());
                        motorLeft2.getCurrentPosition();
                        motorRight2.getCurrentPosition();
                telemetry.update();
            }

            // Stop all motion;
            motorLeft.setPower(0);
            motorRight.setPower(0);
            motorLeft2.setPower(0);
            motorRight2.setPower(0);

            // Turn off RUN_TO_POSITION
            motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeft2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRight2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
        }
    }
}