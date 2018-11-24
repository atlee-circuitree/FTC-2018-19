package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    /* Public OpMode members. */

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

    public DistanceSensor sensorRange = null;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public hardware2018() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
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
        armCombineServo = (ServoImplEx)hwMap.get(Servo.class, "armCombineServo");
        armReleaseServo = (ServoImplEx)hwMap.get(Servo.class, "armReleaseServo");
        sensorRange = hwMap.get(DistanceSensor.class, "distanceLeft");

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

        armJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void StopAll()
    {
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

    public void ResetEncoders()
    {
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
}
