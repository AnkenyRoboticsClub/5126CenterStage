package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp (name="FieldCentric_TeleOp", group="Linear OpMode")
public class FieldCentric_TeleOp extends LinearOpMode {

    static final double TURN_SPEED = 0.5;
    static final double MOVING_SPEED = 0.5;
    static final double MOVING_SPEED_SLOW = 0.25;
    static final double ARM_COUNTS_PER_MOTOR_REV = 288;
    static final double DEGREES_PER_TICK = (360 / ARM_COUNTS_PER_MOTOR_REV);
    private CRServo claw;
    private DcMotor arm;
    private DcMotor armBoost;


    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("Motor3");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("Motor2");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("Motor0");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("Motor1");
        claw = hardwareMap.get(CRServo.class, "claw");
        arm = hardwareMap.get(DcMotor.class, "arm"); //Core ex motor
        armBoost = hardwareMap.get(DcMotor.class, "armBoost");

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int position = arm.getCurrentPosition();
        int armMin = 0;
        int armMax = 705;
        double power = 0;
        double armPower = 0;


        //Reverse the motors that are reversed
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.

            if (gamepad1.options) {
                imu.resetYaw();
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing


            if(gamepad1.left_bumper)
            {
                rx *= TURN_SPEED;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + (rx)) / denominator;
            double backLeftPower = (rotY - rotX + (rx)) / denominator;
            double frontRightPower = (rotY - rotX - (rx)) / denominator;
            double backRightPower = (rotY + rotX - (rx)) / denominator;

            if(!(gamepad1.right_trigger > 0.1)) // Speed is decreased by Moving_Speed unless the a button is pressed down
            {
                frontLeftPower *= MOVING_SPEED;
                backLeftPower *= MOVING_SPEED;
                frontRightPower *= MOVING_SPEED;
                backRightPower *= MOVING_SPEED;
            }
            if((gamepad1.left_trigger > 0.1)) // Speed is decreased by Moving_Speed unless the a button is pressed down
            {
                frontLeftPower *= MOVING_SPEED_SLOW;
                backLeftPower *= MOVING_SPEED_SLOW;
                frontRightPower *= MOVING_SPEED_SLOW;
                backRightPower *= MOVING_SPEED_SLOW;
            }

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            /*
                One all the way down
                Picking up posiotion
                Right trigger to open and close the claw - holding it is closed
                Joystick
                    Up - moves claw twoards closed position
                    Down - moves claw twoards open
             */
            //Claw stuff

            claw.setPower(gamepad2.right_trigger * -2 + 0.7);

            if (-gamepad2.left_stick_y > 0.5){
                position = arm.getCurrentPosition() + 50;
                power = 1;
            } else if (-gamepad2.left_stick_y < -0.5){
                position = arm.getCurrentPosition() - 50;
                power = 1;
            } else{
                power = 0;
            }

            if (gamepad2.left_bumper) {
                //armMax = arm.getCurrentPosition();
            } else {
                if (position < armMin){
                    position = armMin;
                } else if (position > armMax){
                    position = armMax;
                }
                if(gamepad2.y){
                    power = 0.2;
                    position = armMax - 235; //470
                }
                if(gamepad2.a){
                    power = 0.2;
                    position = armMax-40; //665
                }
            }
            if((!(gamepad2.y || gamepad2.a)) && (position > armMin + 60) && (position < armMax - 60)){
                armPower = -gamepad2.left_stick_y * gamepad2.left_trigger;
            }
            else{
                armPower = 0;
            }
            armBoost.setPower(armPower);
            // ((DcMotorEx) arm).setTargetPositionTolerance(20);
            arm.setPower(power);
            arm.setTargetPosition(position);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ((DcMotorEx) arm).setVelocity(1000);

            telemetry.addData("Arm position", arm.getCurrentPosition());
            telemetry.addData("Arm setpoint", position);
            telemetry.addData("Claw position", claw.getDirection());
            telemetry.addData("Motor","%.2f, %.2f, %.2f, %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("IMU: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
    }
}