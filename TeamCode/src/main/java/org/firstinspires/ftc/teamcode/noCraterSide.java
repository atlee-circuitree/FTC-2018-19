

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

// code based on BergTest through the copy/paste paradime (Im saying dont use this). EDIT: looks like we are using this.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Drives arm motors with a controller for testing and positioning the robot for measuring
 */

@Autonomous(name = "No_crater_side_auto", group = "Linear Opmode")

public class noCraterSide extends LinearOpMode {


    // Declare OpMode members.
    hardware2018 robot = new hardware2018();
    MineralDetector mineralDetector = new MineralDetector();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(this, hardwareMap);
        robot.ResetEncoders();

        mineralDetector.init(hardwareMap, telemetry);


        //robot.armCombineServo.setPosition(0.5);

        MineralDetector.MineralPosition goldPosition = MineralDetector.MineralPosition.Unknown;
        MineralDetector.MineralPosition tempPosition = MineralDetector.MineralPosition.Unknown;
        // Wait for the game to start (driver presses PLAY)
        while (!isStarted()) {
            tempPosition = mineralDetector.GetMineralPosition(); //read mineral position before we drop
            if (tempPosition != MineralDetector.MineralPosition.Unknown) {
                goldPosition = tempPosition;
            }
        }
        //waitForStart();

        telemetry.update();
        runtime.reset();

        // Setup a variable for each drive wheel to save power level for telemetry
	//Does not appear to be used for anything right now, has been moved to hardware2018
        double leftPower;
        double rightPower;
        double combineSpeed = 0;
        double armCombineOpenEndTime = 0;

        //drops robot

        boolean dropStageCompleted = false;
        int dropPosition = 21000;
        int jointRaisePosition = 1600;  //1400 - better height for collecting
        int extendOutPosition = 12000;

        //run climb motor until we've dropped
        while (opModeIsActive() && robot.climbMotor.getCurrentPosition() < dropPosition) {
            robot.climbMotor.setPower(1);
        }
        robot.StopAll();

        //Timing based forward movement
        runtime.reset();

        robot.DriveTimed(DriveDirection.Forward, 200);

        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < 3000){
                robot.armReleaseServo.setPosition(0);
        }
        robot.armReleaseServo.setPwmDisable();
        runtime.reset();


        //TODO:move this to it's own file to provide same code for the not crater
        //the dropping part
        while (opModeIsActive() && !dropStageCompleted) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Gold Position", goldPosition);



            if (robot.armJointMotor.getCurrentPosition() >= jointRaisePosition
                    && robot.armExtendMotor.getCurrentPosition() >= extendOutPosition
                    && robot.climbMotor.getCurrentPosition() >= dropPosition) {
                dropStageCompleted = true;
            }


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
        //robot.armCombineServo.setPosition(0.7);


        telemetry.addData("Gold Position", goldPosition);
        telemetry.update();

        robot.StopAll();

	
        runtime.reset();
        robot.DriveTimed(DriveDirection.Forward, 400);

        //lower arm for depositing the marker
        while (robot.armJointMotor.getCurrentPosition() >= 1300)
        {
            robot.ArmJointDrop();
        }
        robot.ArmJointStop();

        robot.armCombineMotor.setPower(.8);
        sleep(1600);
        robot.CombineStop();
        robot.DriveTimed(DriveDirection.Backward, 200);

        while (robot.armJointMotor.getCurrentPosition() <= 1400)
            {
                robot.ArmJointRaise();
            }
            robot.ArmJointStop();


        runtime.reset();
        while(runtime.milliseconds() < 4000) {
            robot.ArmExtendIn();
        }

        if (goldPosition == MineralDetector.MineralPosition.Left) {
            robot.DriveTimed(DriveDirection.Left, 400);
            robot.DriveTimed(DriveDirection.Forward, 1000);
            robot.DriveTimed(DriveDirection.Backward, 1000);
            robot.DriveTimed(DriveDirection.Right, 400);
        } else if (goldPosition == MineralDetector.MineralPosition.Right) {
            robot.DriveTimed(DriveDirection.Right, 400);
            robot.DriveTimed(DriveDirection.Forward, 1000);
            robot.DriveTimed(DriveDirection.Backward, 1000);
            robot.DriveTimed(DriveDirection.Left, 400);
        } else if (goldPosition == MineralDetector.MineralPosition.Center) //golkd center
        {
            robot.DriveTimed(DriveDirection.Forward, 900);
            robot.DriveTimed(DriveDirection.Backward, 900);
        } else
        {
           robot.DriveTimed(DriveDirection.Forward, 900);
           robot.DriveTimed(DriveDirection.Backward, 900);
        }
	
	robot.Rotate(45,.5);
	robot.DriveForwardCheckObstruction(2000);
	robot.DriveTimed(DriveDirection.Backward, 200);
	robot.Rotate(90,.5);
	robot.DriveTimed(DriveDirection.Forward, 1000); 
	    
        robot.StopAll();
        mineralDetector.Shutdown();
    }
}
