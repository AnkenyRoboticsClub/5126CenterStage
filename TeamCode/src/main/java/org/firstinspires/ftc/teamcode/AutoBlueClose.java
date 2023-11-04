package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Auto BlueClose", group="Robot")
//@Disabled
public class AutoBlueClose extends LinearOpMode{
    static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per revolution
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing
    static final double WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For 96 mm diameter - If 140mm use 5.51181
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private CRServo claw;
    BHI260IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode(){
        //Initializes the motors
        frontLeft = hardwareMap.dcMotor.get("Motor3");
        backLeft = hardwareMap.dcMotor.get("Motor2");
        frontRight = hardwareMap.dcMotor.get("Motor0");
        backRight = hardwareMap.dcMotor.get("Motor1");
        claw = hardwareMap.get(CRServo.class, "claw");

        //Reverse the motors that need to be reversed
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Resets the encoders?
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Starts the encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Sets up the gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        IMU.Parameters myIMUparameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(myIMUparameters);

        //claw.setPower(0.2);

        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            moveDistance(0.5, 6);
            turn(90);
            moveDistance(0.5,26);
            //claw.setPower(0);
            return;
        }
    }

    //Moves the robot forward a distance (in inches)
    public void moveDistance(double power, int distance){
        //Resets the encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Sets the target position to the distance
        frontLeft.setTargetPosition(distance * (int) COUNTS_PER_INCH);
        backLeft.setTargetPosition(distance * (int) COUNTS_PER_INCH);
        frontRight.setTargetPosition(distance * (int) COUNTS_PER_INCH);
        backRight.setTargetPosition(distance * (int) COUNTS_PER_INCH);

        //Takes motors to that position
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Goes forward at the certain speed
        setAllPower(power);

        //Waits until the motors are done moving
        while(frontLeft.isBusy() && backLeft.isBusy() && frontRight.isBusy() && backRight.isBusy()){

        }

        //Stops the motors
        stopMotor();

        //Goes back to running using the encoder
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setAllPower(double power) {
        setMotorPower(power, power, power, power);
    }
    public void stopMotor(){
        setMotorPower(0, 0, 0, 0);
    }
    public void setMotorPower(double frontL, double backL, double frontR, double backR){
        frontLeft.setPower(frontL);
        backLeft.setPower(backL);
        frontRight.setPower(frontR);
        backRight.setPower(backR);
    }
    public void resetAngle(){
        //Need to change the axes order based on orientation of the extension hub
        //Look up IMU interface
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle(){
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        //delta Angle is only between -179 and 180
        if (deltaAngle > 180){
            deltaAngle -= 360;
        } else if (deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        telemetry.update();
        return currAngle;
    }
    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 2){
            //Sets power to 0.3 depending on if the error is negative or positive(right or left)
            double motorPower = (error < 0 ? -0.3 : 0.3);
            //Left is negative right is positive
            setMotorPower(-motorPower, -motorPower, motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();

        }

        stopMotor();
    }
    public void turnTo(double degrees){
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degrees - orientation.firstAngle;

        if(error > 180){
            error -= 360;
        } else if (error < -180){
            error += 360;
        }

        turn(error);
    }

}
