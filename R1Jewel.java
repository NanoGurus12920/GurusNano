/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "R1Jewel", group = "Pushbot")
//@Disabled
public class R1Jewel extends LinearOpMode {

    /* Declare OpMode members. */
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;

    ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor leftEscalator;
    private DcMotor rightEscalator;
    private Servo largeJewelArm   = null;
    private Servo smallJewelArm = null;
    ColorSensor color_sensor;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftEscalator = hardwareMap.dcMotor.get("leftEscalator");
        rightEscalator = hardwareMap.dcMotor.get("rightEscalator");
        largeJewelArm = hardwareMap.get(Servo.class, "leftarm");
        smallJewelArm = hardwareMap.get(Servo.class, "rightarm");
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        color_sensor.enableLed(true);

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            armDown(2);
            jewel(2);
            armUp(2);
            grabGlyph(1);
            gotoSafety();
            dropGlyph(1);
            break;// stop
        }
        stop();
    }

    /**
     * Bringing both the arms down to prepare for using the color sensor
     * @param holdTime
     */
    public void armDown(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time() < holdTime) {
            smallJewelArm.setPosition(0.45);  // Move up the small arm - the way its installed, starting position is 0. facing front
            largeJewelArm.setPosition(0.03);  // Move down - the way it's installed, starting position is 0.5, facing up
        }

    }

    /**
     * The color sensor is reading the value of the red jewel, and knocking off the the opposite color jewel
     * @param holdTime
     */
    public void jewel(double holdTime){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        int redValue= color_sensor.red();
        int blueValue= color_sensor.blue();
        while (opModeIsActive() && holdTimer.time() < holdTime) {
            telemetry.addData("Red  ", redValue);
            telemetry.addData("Blue ", blueValue);
            telemetry.update();

            if (redValue>blueValue){    //Red jewel detected, knock off blue jewel using small arm
                smallJewelArm.setPosition(1);
            }else{                      //Blue jewel detected, knock it off using small arm
                smallJewelArm.setPosition(0);
            }
        }
    }

    /**
     * Bringing both the arms up so they dont interfere during the rest of the game
     * @param holdTime
     */
    public void armUp(double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();


        while (opModeIsActive() && holdTimer.time() < holdTime) {
            //smallJewelArm.setPosition(0);  //this servo needs to go up 180 degrees
            largeJewelArm.setPosition(0.5);  //this servo needs to go up 90 degrees
        }
    }

    /**
     * Grabbing the glyph for one second
     * @param holdTime
     */
    public void grabGlyph(double holdTime) {
        runEscalator(1);
    }

    /**
     * Maneuvering the bot to get to the safety zone
     */
    public void gotoSafety(){
        while (opModeIsActive()) {
            encoderDrive(DRIVE_SPEED, -20, -20, 6.0);  // S1: Go backwards 20 Inches with 6 Sec timeout
            encoderDrive(TURN_SPEED, 12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, -2, -2, 2.0);  // S3: Backward 2 Inches with 2 Sec timeout
            break;// stop
        }
    }

    /**
     * Dropping the glyph into the cryptobox
     * @param holdTime
     */
    public void dropGlyph(double holdTime) {
        runEscalator(1);
    }

    /**
     * Runs the escalator for given number of seconds
     * @param seconds
     */
    private void runEscalator(double seconds){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time() < seconds) {
            leftEscalator.setPower(0.7);
            rightEscalator.setPower(-0.7);
        }
        leftEscalator.setPower(0);
        rightEscalator.setPower(0);
    }

    /**
     * this is the program that moves the bot using encoders
     * @param speed two parameters (drive and turn speed), 0-1 range for motors
     * @param leftInches the distance we want our left motor / wheel to travel
     * @param rightInches the distance we want our right motor / wheel to travel
     * @param timeoutS
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

}