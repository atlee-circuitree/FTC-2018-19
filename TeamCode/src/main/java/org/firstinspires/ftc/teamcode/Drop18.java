package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This is NOT an opmode.
 * This class can be used to drop the 2018 bot
 */




public class Drop18 {

    hardware2018 robot = null;
    MineralDetector mineralDetector = null;
    Telemetry telemetry = null;
    private ElapsedTime runtime = new ElapsedTime();
    LinearOpMode opMode = null;

    /* Constructor */
    public Drop18(hardware2018 robotParam, MineralDetector mineralParam, Telemetry telemetryParam, LinearOpMode opModeParam) {
        robot = robotParam;
        telemetry = telemetryParam;
        opMode = opModeParam;

    }

    public void dropBot(){


	    boolean dropStageCompleted = false;
	    int dropPosition = 21000;
        int jointRaisePosition = 1800;  //1400 - better height for collecting
        int extendOutPosition = 5000;
	
        //run climb motor until we've dropped
        while (robot.climbMotor.getCurrentPosition() < dropPosition) {
            robot.climbMotor.setPower(1);
            }
        robot.StopAll();

        robot.DriveTimed(DriveDirection.Forward, 200);

        runtime.reset();

        
	//the dropping part

            while (opMode.opModeIsActive() && !dropStageCompleted) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());

                if (robot.armJointMotor.getCurrentPosition() >= jointRaisePosition
                        && robot.armExtendMotor.getCurrentPosition() >= extendOutPosition
                        && robot.climbMotor.getCurrentPosition() >= dropPosition) {
                    dropStageCompleted = true;
                }

                // unfurl arm
                if (runtime.milliseconds() < 1000)
                    robot.armReleaseServo.setPosition(0);
                else
                    robot.armReleaseServo.setPwmDisable();

                //raise arm
                if (robot.armJointMotor.getCurrentPosition() < jointRaisePosition)
                    robot.ArmJointRaise();
                else
                    robot.ArmJointStop();

                //extend arm (should be disabled)
              /*  if (runtime.milliseconds() > 2000 && robot.armExtendMotor.getCurrentPosition() < extendOutPosition)
                    robot.ArmExtendOut();
                else
                    robot.ArmExtendStop();
                */
                // Show the elapsed game time and wheel power.
                telemetry.update();
            }

            runtime.reset();
            while(opMode.opModeIsActive() && runtime.milliseconds() < 1000)
            {
                robot.armReleaseServo.setPosition(0);
            }
            robot.armReleaseServo.setPwmDisable();

        telemetry.update();

        robot.StopAll();
	//end of dropping

    }
}
