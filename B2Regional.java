
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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

@Autonomous(name = "B2FullAuto", group = "Pushbot")
//@Disabled
public class B2Regional extends LinearOpMode {

    /* Declare OpMode members. */
    ElapsedTime runtime = new ElapsedTime();
    HardwareAuto robot = new HardwareAuto();
    ColorSensor color_sensor;
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;
    OpenGLMatrix lastLocation = null;
    double tX;
    double tY;
    double tZ;
    double rX;
    double rY;
    double rZ;
    VuforiaLocalizer vuforia;
    VuforiaTrackable relicTemplate;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        color_sensor.enableLed(true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AQg97OD/////AAAAGSDjZA+eGkd2gHbTl7kt1QB3wX/cq0qTsvj0FonpkRao8qy+XeqpK4zKIcQCW2QZJumCijTbg+jQF9FYMR+5l/VGJrjJzLl7RIbQTxhIVtxGgzj2nnHao8V7MtDvNdjK68wF5h7w4TwHHRhRDW4de8N87co3FMksjTVBxGKtEUlXeZn1Lcy5dkTpSm1skfAMxZX6j4hzp8B+ISM28CAwx90fOOYTvZnF82y7T2XqNlBfwXm9as/CDYy5Zw+ARMhPSit7VRKOQw6WRSJ0tZXt7yJcq9XHIjLFnU/reRrhx9q6RdyLnrGeiFK6HjgxOBertINXJhgJUquCzunWOeMOKxW8ut6Iw1AU9kxIjMhVbw/b";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);

        waitForStart();
        relicTrackables.activate();
        while (opModeIsActive()) {
            armDown(2);
            jewel(2);
            armUp(2);
            //grabGlyph(1);
            //gotoSafety(12);
            Vuforia(12);
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
            robot.smallJewelArm.setPosition(0.45);  // Move up the small arm - the way its installed, starting position is 0. facing front
            robot.largeJewelArm.setPosition(0.85);  // Move down - the way it's installed, starting position is 0.5, facing up
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

            if (redValue<blueValue){    //Red jewel detected, knock off blue jewel using small arm
                robot.smallJewelArm.setPosition(1);
            }else{                      //Blue jewel detected, knock it off using small arm
                robot.smallJewelArm.setPosition(0);
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
            robot.largeJewelArm.setPosition(0.3);  //this servo needs to go up 90 degrees
        }
    }

    /**
     * Grabbing the glyph for one second
     * @param holdTime
     */
    public void grabGlyph(double holdTime) {
        runEscalator(0.5);
    }

    /**
     * Maneuvering the bot to get to the safety zone
     */
    public void gotoSafety(double seconds){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        while (opModeIsActive()&& holdTimer.time() < seconds) {
            encoderDrive(DRIVE_SPEED, -20, -20, 6.0);  // S1: Go backwards 20 Inches with 6 Sec timeout
            encoderDrive(TURN_SPEED, -9, 9, 4.0);  // S2: Turn Right 9 Inches with 4 Sec timeout
            encoderDrive(DRIVE_SPEED, -5, -5, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
            encoderDrive(TURN_SPEED, 9, -9, 4.0);  // S3: Backward 10 Inches with 2 Sec timeout
            encoderDrive(DRIVE_SPEED, -4, -4, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
            break;// stop
        }
    }

    /**
     * Dropping the glyph into the cryptobox
     * @param holdTime
     */
    public void dropGlyph(double holdTime) {
        runEscalator(3);
        encoderDrive(DRIVE_SPEED, -5, -5, 2.0);  // S4: Backward 1 Inch with 2 Sec timeout
        encoderDrive(DRIVE_SPEED, 4, 4, 2.0);  // S5: Forward 2 Inch with 2 Sec timeout
    }

    /**
     * Runs the escalator for given number of seconds
     * @param seconds
     */
    private void runEscalator(double seconds){
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time() < seconds) {
            robot.leftEscalator.setPower(0.7);
            robot.rightEscalator.setPower(-0.7);
        }
        robot.leftEscalator.setPower(0);
        robot.rightEscalator.setPower(0);
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
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d", robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void Vuforia(double seconds)
    {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.time() < seconds) {
            robot.smallJewelArm.setPosition(0.45);  //this servo needs to go up 180 degrees
            encoderDrive(0.125, -4.5, -4.5, 7.0); // Go Backward 5 inches in order to read the Vuforia Picture
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark Value is =",vuMark);
            telemetry.update();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);
                    rX = rot.firstAngle;
                    rY = rot.secondAngle;
                    rZ = rot.thirdAngle;
                }
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    telemetry.addData("Vumark is", "LEFT");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    encoderDrive(0.6, 22.5, 22.5, 6.0);  // S1: Go backwards 20 Inches with 6 Sec timeout
                    encoderDrive(TURN_SPEED, -9.0, 9.0, 4.0);  // S2: Turn Right 9 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -5, -5, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                    encoderDrive(TURN_SPEED, -9, 9, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                    encoderDrive(DRIVE_SPEED, -4, -4, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                    break;// stop
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    telemetry.addData("Vumark is", "RIGHT");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    encoderDrive(0.6, 31, 31, 6.0);  // S1: Go backwards 20 Inches with 6 Sec timeout
                    encoderDrive(TURN_SPEED, -9, 9, 4.0);  // S2: Turn left 9 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -11, -11, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                    encoderDrive(TURN_SPEED, -9, 9, 4.0);  // S4: Turn right 9 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -4, -4, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                    break; // stop
                } else if (vuMark == RelicRecoveryVuMark.CENTER)
                {
                    telemetry.addData("Vumark is", "CENTER");
                    telemetry.addData("X =", tX);
                    telemetry.addData("Y =", tY);
                    telemetry.addData("Z =", tZ);
                    encoderDrive(0.6, 22.5, 22.5, 6.0);  // S1: Go backwards 20 Inches with 6 Sec timeout
                    encoderDrive(TURN_SPEED, -9, 9, 4.0);  // S2: Turn left 9 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -16.5, -16.5, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                    encoderDrive(TURN_SPEED, -9, 9, 4.0);  // S4: Turn right 9 Inches with 4 Sec timeout
                    encoderDrive(DRIVE_SPEED, -4, -4, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                    break; // stop
                }
            }
            else{
                encoderDrive(0.6, 31, 31, 6.0);  // S1: Go backwards 20 Inches with 6 Sec timeout
                encoderDrive(TURN_SPEED, -9, 9, 4.0);  // S2: Turn Right 9 Inches with 4 Sec timeout
                encoderDrive(DRIVE_SPEED, -12, -12, 2.0);  // S3: Backward 10 Inches with 2 Sec timeout
                encoderDrive(TURN_SPEED, -9, 9, 4.0);  // S2: Turn Right 9 Inches with 4 Sec timeout
                encoderDrive(DRIVE_SPEED, -2.5, -2.5, 4.0);  // S2: backward 4 inches
                break;
            }
        }
    }
}