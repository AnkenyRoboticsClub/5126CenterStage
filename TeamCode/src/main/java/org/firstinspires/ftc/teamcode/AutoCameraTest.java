/* Copyright (c) 2019 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Red Alliance Backstage", group = "Concept")
//Disabled
public class AutoCameraTest extends LinearOpMode {

    final double DESIRED_DISTANCE = 10; // how close the camera should get to the object (inches)

    final double SPEED_GAIN  =  0.03  ;   // Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   // Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   // Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor arm;
    private CRServo claw;


    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private int DESIRED_TAG_ID = 0;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/RedCube.tflite";
    private static final String[] LABELS = {
            "RedCube1"
    };
    private int myExposure;

    //arm variables
    static final int MIN_ARM_POSITION = 0;
    static final int MAX_ARM_POSITION = 1390;

    static final int ARM_ANGLE_POSITION_FROM_MAX = 105;


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;


    @Override
    public void runOpMode() {

        //Variables
        boolean targetFound = false; //Set to true when an AprilTag target is detected
        boolean pixelFound = false; //Set to true when pixel is found on spike mark
        String pixelLocation = ""; //Set to center, left, or right spike mark
        double drive = 0; //forward speed
        double strafe = 0; //Strafe speed
        double turn = 0; //turning speed
        int currentStep = 1;
        ElapsedTime runtime = new ElapsedTime();

        initTfod();
        //Initalizes the motors
        frontLeft = hardwareMap.dcMotor.get("Motor3");
        backLeft = hardwareMap.dcMotor.get("Motor2");
        frontRight = hardwareMap.dcMotor.get("Motor0");
        backRight = hardwareMap.dcMotor.get("Motor1");
        claw = hardwareMap.get(CRServo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm");

        //Reverses motors
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Arm motor
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int position = arm.getCurrentPosition();

        //Claw closes
        claw.setPower(0.2);

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        runtime.reset(); //starts the timer
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                targetFound = false;
                desiredTag = null;

                //Step 1 - move forward
                if(currentStep == 1){
                    if(runtime.milliseconds() < 450){
                        moveRobot(10, 0, 0);
                    }
                    else{
                        stopRobot();
                        currentStep = 2;
                        runtime.reset();
                    }
                }

                //Step 2 - turn to look at left spike
                if(currentStep == 2){
                    if(runtime.milliseconds() < 175){
                        moveRobot(0, 0, 5);
                    }
                    else{
                        stopRobot();
                        currentStep = 3;
                        runtime.reset();
                    }
                }

                //Step 3 - use Tensorflow to check for pixel on left spike
                if (currentStep == 3) {
                    if(runtime.milliseconds() < 3000){
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        //go through list of recognitions and look for pixel
                        for(Recognition recognition : currentRecognitions){
                            if(recognition.getLabel() == "Pixel"){
                                currentStep = 4;
                                pixelLocation = "left";
                                pixelFound = true;
                                runtime.reset();
                            }
                            else {
                                sleep(50);
                            }
                        }
                    }
                    else {
                        currentStep = 7; //try center mark
                        runtime.reset();
                    }
                }

                //Step 4 - move forward towards left spike mark
                if(currentStep == 4){
                    if (runtime.milliseconds() < 460){
                        moveRobot(5, 0, 0);
                    }
                    else {
                        stopRobot();
                        currentStep = 5;
                        runtime.reset();
                    }
                }

                //Step 5 - back away from spike mark to drop off purple pixel
                if(currentStep == 5){
                    if (runtime.milliseconds() < 460){
                        moveRobot(-5,0,0);
                    }
                    else{
                        stopRobot();
                        currentStep = 6; //Point towards backdrop from left spike
                        runtime.reset();
                    }
                }

                //Step 6 - point towards backdrop from left mark
                if(currentStep == 6){
                    if(runtime.milliseconds() < 750){
                        moveRobot(0,0,-5);
                    }
                    else{
                        stopRobot();
                        currentStep = 20;
                        //runtime.reset();
                    }
                }

                //Step 7 - rotation clockwise to check center spike mark
                if (currentStep == 7){
                    if(runtime.milliseconds() < 125){
                        moveRobot(0,0,-5);
                    }
                    else{
                        stopRobot();
                        currentStep = 8;
                        runtime.reset();
                    }
                }

                //Step 8 - check for pixel on center spike mark
                if (currentStep == 8) {
                    if(runtime.milliseconds() < 3000){
                        List<Recognition> currentRecognitions = tfod.getRecognitions();
                        //go through list of recognitions and look for pixel
                        for(Recognition recognition : currentRecognitions){
                            if(recognition.getLabel() == "Pixel"){
                                currentStep = 9;
                                pixelLocation = "center";
                                pixelFound = true;
                                runtime.reset();
                            }
                            else {
                                sleep(50);
                            }
                        }
                    }
                    else {
                        currentStep = 12; //try right mark
                        runtime.reset();
                    }
                }

                //Step 9 - move forward towards center spike mark
                if(currentStep == 9){
                    if (runtime.milliseconds() < 500){
                        moveRobot(5,0,0);
                    }
                    else{
                        stopRobot();
                        currentStep = 9;
                        runtime.reset();
                    }
                }

                //Step 10 - back away from center mark, dropping off purple pixel
                if(currentStep == 10){
                    if (runtime.milliseconds() < 500){
                        moveRobot(-5,0,0);
                    }
                    else{
                        stopRobot();
                        currentStep = 11;
                        runtime.reset();
                    }
                }

                //Step 11 - point towards backdrop from center mark
                if(currentStep == 11){
                    if (runtime.milliseconds() < 500){
                        moveRobot(0,0,-5);
                    }
                    else{
                        stopRobot();
                        currentStep = 20;
                        // runtime.reset();
                    }
                }

                //Step 12 - turn towards right spike mark
                if(currentStep == 12){
                    if (runtime.milliseconds() < 200){
                        moveRobot(0,0,-5);
                    }
                    else{
                        stopRobot();
                        currentStep = 13;
                        runtime.reset();
                    }
                }

                //Step 13 - move forward towards right spike mark
                if(currentStep == 13){
                    if(runtime.milliseconds() < 480){
                        moveRobot(5,0,0);
                    }
                    else {
                        stopRobot();
                        currentStep = 14;
                        runtime.reset();
                    }
                }

                //Step 14 - move backward from right spike
                if(currentStep == 14){
                    if(runtime.milliseconds() < 480){
                        moveRobot(-5,0,0);
                    }
                    else{
                        stopRobot();
                        currentStep = 15;
                        runtime.reset();
                    }
                }

                //Step 15 - point towards backdrop from right spike
                if(currentStep == 15){
                    if(runtime.milliseconds() < 250){
                        moveRobot(0,0,-5);
                    }
                    else{
                        stopRobot();
                        currentStep = 20;
                        // runtime.reset();
                    }
                }

                //Step 20 - set up april tags
                if(currentStep == 20) {
                    visionPortal.close();
                    sleep(50);
                    //Start up april tag
                    initAprilTag();
                    sleep(50);
                    setManualExposure(20, 250); //reduce motion blur

                    if (pixelLocation == "left") {
                        DESIRED_TAG_ID = 4;
                    } else if (pixelLocation == "center") {
                        DESIRED_TAG_ID = 5;
                    } else {
                        DESIRED_TAG_ID = 6;
                    }

                    currentStep = 21;
                }

                //Step 21 - move to backdrop using april tag
                if(currentStep == 21){
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections){
                        if((detection.metadata != null) && (detection.id == DESIRED_TAG_ID)){
                            targetFound = true;
                            desiredTag = detection;
                            break; //dont look any futher
                        }
                    }

                    if(targetFound){
                        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                        double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double  headingError    = desiredTag.ftcPose.bearing;
                        double  yawError        = desiredTag.ftcPose.yaw;
                        if ((rangeError < 7) && (Math.abs(headingError) < 5) && (Math.abs(yawError) < 5)){
                            //If close stop
                            drive = 0;
                            turn = 0;
                            strafe = 0;
                            currentStep = 22;
                        }
                        else{
                            // Use the speed and turn "gains" to calculate how we want the robot to move.
                            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                            telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                        }

                        // Apply desired axes motions to the robot
                        moveRobot(drive, strafe, turn);
                        sleep(10);
                    }
                    else {
                        stopRobot();
                        sleep(10);
                    }
                    runtime.reset();
                }

                //Step 22 - May need an adjust step
                if(currentStep == 22){
                    currentStep = 23;
                    runtime.reset();
                }
                //Step 23 - Turn robot 90 degrees clockwise
                if (currentStep == 23){
                    if(runtime.milliseconds() < 500){
                        moveRobot(0,0,-5);
                    }
                    else{
                        stopRobot();
                        currentStep = 24;
                    }
                }

                //Step 24 - raise arm to drop position
                if (currentStep == 24){
                    arm.setPower(1);
                    arm.setTargetPosition(MAX_ARM_POSITION - ARM_ANGLE_POSITION_FROM_MAX);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(arm.isBusy()){

                    }
                    sleep(10);
                    claw.setPower(0); //drops pixel
                    sleep(10);
                    currentStep = 25;
                }
                //Step 25 - lower arm
                if(currentStep == 25){
                    arm.setTargetPosition(MIN_ARM_POSITION);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while(arm.isBusy()){

                    }
                    sleep(10);
                    currentStep = 26;
                    runtime.reset();
                }
                //Step 26 - move towards wall
                if(currentStep == 26){
                    if(runtime.milliseconds() < 400){
                        moveRobot(5,0,0);
                    }
                    else{
                        stopRobot();
                        sleep(10);
                    }
                }
                //Step 27 - may need to rotate to a good position
                // Push telemetry to the Driver Station.
                telemetry.addData("current step", currentStep);
                telemetry.addData("pixel found", pixelFound);
                telemetry.addData("pixel location", "%s", pixelLocation);
                telemetry.addData("tag target", DESIRED_TAG_ID);
                telemetry.addData("tag found", targetFound);
                telemetry.update();

            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // Use setModelAssetName() if the TF Model is built in as an asset.
            // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            //.setModelAssetName(TFOD_MODEL_ASSET)
            .setModelFileName(TFOD_MODEL_FILE)

            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

    private void moveRobot(double x, double y, double yaw){
        double frontLeftPower = x-y-yaw;
        double frontRightPower = x+y+yaw;
        double backLeftPower = x+y-yaw;
        double backRightPower = x-y+yaw;

        //powers less than 1
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        //Sets power to teh wheels
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

    }
    private void stopRobot(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
    private void setManualExposure(int exposureMS, int gain){
        if(visionPortal == null){
            return;
        }

        if(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while(!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)){
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        //Honestly I have no idea what this does
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

}   // end class
