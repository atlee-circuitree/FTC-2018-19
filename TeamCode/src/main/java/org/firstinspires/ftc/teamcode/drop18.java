package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 * This class can be used to drop the 2018 bot
 */


//TODO: make thing work

public class drop18 {

    hardware2018 robot = null;
    MineralDetector mineralDetector = null;
    telemetry
    private ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public drop18(hardware2018 robotParam, MineralDetector mineralParam) {
        robot = robotParam;
        mineralDetector = mineralParam;
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
        if (goldPosition == MineralDetector.MineralPosition.Center) { 
            while (!dropStageCompleted) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Gold Position", goldPosition);

                if (robot.armJointMotor.getCurrentPosition() >= jointRaisePosition
                        && robot.armExtendMotor.getCurrentPosition() >= extendOutPosition
                        && robot.climbMotor.getCurrentPosition() >= dropPosition) {
                    dropStageCompleted = true;
                }

                if (runtime.milliseconds() < 1000)
                    robot.armReleaseServo.setPosition(0);
                else
                    robot.armReleaseServo.setPwmDisable();

                if (robot.armJointMotor.getCurrentPosition() < jointRaisePosition)
                    robot.ArmJointRaise();
                else
                    robot.ArmJointStop();

                if (robot.armExtendMotor.getCurrentPosition() < extendOutPosition)
                    robot.ArmExtendOut();
                else
                    robot.ArmExtendStop();

                // Show the elapsed game time and wheel power.
                telemetry.update();
            } 
        } else {
            while (opModeIsActive() && !dropStageCompleted) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Gold Position", goldPosition);

                if (robot.armJointMotor.getCurrentPosition() >= jointRaisePosition
                        && robot.armExtendMotor.getCurrentPosition() >= extendOutPosition
                        && robot.climbMotor.getCurrentPosition() >= dropPosition) {
                    dropStageCompleted = true;
                }

                if (runtime.milliseconds() < 1000)
                    robot.armReleaseServo.setPosition(0);
                else
                    robot.armReleaseServo.setPwmDisable();

                if (robot.armJointMotor.getCurrentPosition() < jointRaisePosition)
                    robot.ArmJointRaise();
                else
                    robot.ArmJointStop();

                if (runtime.milliseconds() > 2000 && robot.armExtendMotor.getCurrentPosition() < extendOutPosition)
                    robot.ArmExtendOut();
                else
                    robot.ArmExtendStop();

                // Show the elapsed game time and wheel power.
                telemetry.update();
            }
        }

        telemetry.addData("Gold Position", goldPosition);
        telemetry.update();

        robot.StopAll();
	//end of dropping

    }
}
