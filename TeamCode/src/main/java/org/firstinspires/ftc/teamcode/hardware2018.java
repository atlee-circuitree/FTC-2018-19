package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Double.isNaN;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is our 2018 bot.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:
 * <p>
 * Note: the configuration of the servos is such that:
 */

public class hardware2018 {
    LinearOpMode opModeObject = null;

    //motors at base that power wheels
    public DcMotor leftDriveFront = null;
    public DcMotor leftDriveRear = null;
    public DcMotor rightDriveFront = null;
    public DcMotor rightDriveRear = null;

    public DcMotor climbMotor = null;

    public DcMotor armExtendMotor = null;
    public DcMotor armJointMotor = null;
    public DcMotor armCombineMotor = null;

    public ServoImplEx armCombineServo = null;
    public ServoImplEx armReleaseServo = null;

    public DistanceSensor sensorRangeLeft = null;
    public DistanceSensor sensorRangeRight = null;
    public DistanceSensor sensorDistance = null;

    // The IMU sensor object
    public BNO055IMU imu = null;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean aButton, bButton, touched;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public hardware2018() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(LinearOpMode currentOpMode, HardwareMap ahwMap) {
        opModeObject = currentOpMode;
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors

        // the motors at the base, hardware names should use the numbering on the robot.
        // Check if this is working correctly
        leftDriveFront = hwMap.get(DcMotor.class, "driveLeftFrontMotor");
        leftDriveRear = hwMap.get(DcMotor.class, "driveLeftRearMotor");
        rightDriveFront = hwMap.get(DcMotor.class, "driveRightFrontMotor");
        rightDriveRear = hwMap.get(DcMotor.class, "driveRightRearMotor");
        climbMotor = hwMap.get(DcMotor.class, "climbMotor");
        armJointMotor = hwMap.get(DcMotor.class, "armJointMotor");
        armExtendMotor = hwMap.get(DcMotor.class, "armExtendMotor");
        armCombineMotor = hwMap.get(DcMotor.class, "armWheelMotor");
        armCombineServo = (ServoImplEx) hwMap.get(Servo.class, "armCombineServo");
        armReleaseServo = (ServoImplEx) hwMap.get(Servo.class, "armReleaseServo");
        sensorRangeLeft = hwMap.get(DistanceSensor.class, "colorLeft");
        sensorRangeRight = hwMap.get(DistanceSensor.class, "colorRight");
        sensorDistance = hwMap.get(DistanceSensor.class, "distance");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        //ensures motors are rotating in the correct direction.  One side must always be reversed
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);

        climbMotor.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        leftDriveFront.setPower(0);
        leftDriveRear.setPower(0);
        rightDriveFront.setPower(0);
        rightDriveRear.setPower(0);
        climbMotor.setPower(0);
        armJointMotor.setPower(0);
        armExtendMotor.setPower(0);
        armCombineMotor.setPower(0);

        //Brake mode on climb and arm motors to prevent creep
        armJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Brake mode on drive motors to make movement more accurate
        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set motors in starting state
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armJointMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armCombineMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Stops all motors - should be used at the end of OpModes
    public void StopAll() {
        // Set all motors to zero power
        leftDriveFront.setPower(0);
        leftDriveRear.setPower(0);
        rightDriveFront.setPower(0);
        rightDriveRear.setPower(0);
        climbMotor.setPower(0);
        armJointMotor.setPower(0);
        armExtendMotor.setPower(0);
        armCombineMotor.setPower(0);
    }

    //Resets encoders to 0
    public void ResetEncoders() {
        // Set all motors to run with encoders.
        // Use RUN_WITHOUT_ENCODER if encoders are not installed or not wanted.
        // We may need to get rid of the run using encoder things because it could be causing problems.
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armCombineMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armJointMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armCombineMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void DriveTimed(DriveDirection direction, int timeInMilliseconds) {
        DriveTimed(direction, timeInMilliseconds, 1);
    }

    public void DriveTimed(DriveDirection direction, int timeInMilliseconds, double Power) {

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (opModeObject.opModeIsActive() && runtime.milliseconds() < timeInMilliseconds) {
            if (direction == DriveDirection.Forward)
                Forward(Power);
            else if (direction == DriveDirection.Backward)
                Backwards(Power);
            else if (direction == DriveDirection.Left)
                Left(Power);
            else if (direction == DriveDirection.Right)
                Right(Power);
        }
        StopDrive();
    }

    //Drives forward until obstructed or max time has passed
    public void DriveForwardCheckObstruction(int maxTimeInMilliseconds) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        Forward();
        while (opModeObject.opModeIsActive() && runtime.milliseconds() < maxTimeInMilliseconds) {
            if(FrontObstructed())
            {
                StopDrive();
                break;
            }
        }
        StopDrive();
    }

    //Drive forward at 100% power
    public void Forward() {
        Forward(1);
    }

    //Drive forward at specified power
    public void Forward(double Power) {
        Power = Math.abs(Power); //make sure it's positive
        leftDriveRear.setPower(Power);
        leftDriveFront.setPower(Power);
        rightDriveRear.setPower(Power);
        rightDriveFront.setPower(Power);
    }

    //Drive backwards at 100% power
    public void Backwards() {
        Backwards(1);
    }

    //Drive backwards at 100% power
    public void Backwards(double Power) {
        Power = -Math.abs(Power); //make number negative so it's backwards
        leftDriveRear.setPower(Power);
        leftDriveFront.setPower(Power);
        rightDriveRear.setPower(Power);
        rightDriveFront.setPower(Power);
    }

    //Stops just the drive motors
    public void StopDrive() {
        leftDriveRear.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveRear.setPower(0);
        rightDriveFront.setPower(0);
    }

    public void Left() {
        Left(1);
    }

    public void Left(double Power) {
        Power = Math.abs(Power);
        leftDriveRear.setPower(-Power);
        leftDriveFront.setPower(-Power);
        rightDriveRear.setPower(Power);
        rightDriveFront.setPower(Power);
    }

    public void Right() {
        Right(1);
    }

    public void Right(double Power) {
        Power = Math.abs(Power);
        leftDriveRear.setPower(Power);
        leftDriveFront.setPower(Power);
        rightDriveRear.setPower(-Power);
        rightDriveFront.setPower(-Power);
    }

    public void ArmJointRaise() {
        armJointMotor.setPower(1);
    }

    public void ArmJointDrop() {
        armJointMotor.setPower(-1);
    }

    public void ArmJointStop() {
        armJointMotor.setPower(0);
    }

    public void ArmExtendOut() {
        armExtendMotor.setPower(1);
    }

    public void ArmExtendIn() {
        armExtendMotor.setPower(-1);
    }

    public void ArmExtendStop() {
        armExtendMotor.setPower(0);
    }

    public void CombineForward() {
        //TODO - double check direction
        armCombineMotor.setPower(1);
    }

    public void CombineReverse() {
        armCombineMotor.setPower(-1);
    }

    public void CombineStop() {
        armCombineMotor.setPower(0);
    }

    public void HopperServoOpen() {
        armCombineServo.setPosition(1);
    }

    public void HopperServoClose() {
        armCombineServo.setPosition(0.6);
    }

    public void HopperServoFlipArm() {
        armCombineServo.setPosition(0);
    }

    public void Rotate(int degrees, double Power) {
        double leftPower, rightPower;

        double adjustment = 0;
        if (power <= 0.5) {
            adjustment = 5.2;
        } else if (power <= 0.8) {
            adjustment = 10;
        } else {
            adjustment = 15;
        }
        //0.5 = 5 degress overshoot
        //0.8 = 10 degrees overshoot
        //1 = 15-20

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            Right(power);
        } else if (degrees > 0) {   // turn left.
            Left(power);
        } else return;


        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeObject.opModeIsActive() && getAngle() == 0) {
            }

            while (opModeObject.opModeIsActive() && getAngle() > (degrees + adjustment)) {
            }
        } else    // left turn.
        {
            while (opModeObject.opModeIsActive() && getAngle() < (degrees - adjustment)) {
            }
        }

        // turn the motors off.
        StopDrive();
        // wait for rotation to stop.
        opModeObject.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private boolean FrontObstructed() {
        boolean obstructed = false;

        double distLeft = sensorRangeLeft.getDistance(DistanceUnit.CM);
        double distRight = sensorRangeRight.getDistance(DistanceUnit.CM);
        double longDistance = sensorDistance.getDistance(DistanceUnit.CM);

        if (longDistance > 15 && longDistance < 30) {
            obstructed = true;
        } else if (longDistance < 15 && !isNaN(distRight) && distRight < 18) {
            obstructed = true;
        } else if (!isNaN(distLeft) && distLeft < 18)
            obstructed = true;

        return obstructed;
    }
}
