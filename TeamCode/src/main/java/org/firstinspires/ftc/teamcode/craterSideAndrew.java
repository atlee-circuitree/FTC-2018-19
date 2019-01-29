

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

@Autonomous(name = "AWOnly_crater_side_auto", group = "Linear Opmode")
//@Disabled
public class craterSideAndrew extends LinearOpMode {


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
        int jointRaisePosition = 2600;  //1400 - better height for collecting
        int extendOutPosition = 0;

        //run climb motor until we've dropped
        while (opModeIsActive() && robot.climbMotor.getCurrentPosition() < dropPosition) {
            robot.climbMotor.setPower(1);
        }
        robot.StopAll();

        //Timing based forward movement
        runtime.reset();

        robot.DriveTimed(DriveDirection.Forward, 200);

        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < 3000) {
            robot.armReleaseServo.setPosition(0);
        }
        robot.armReleaseServo.setPwmDisable();
        runtime.reset();

        while (opModeIsActive() && robot.armJointMotor.getCurrentPosition() < 2800) {
            robot.ArmJointRaise();
        }
        robot.ArmJointStop();


        dropStageCompleted = true;
        boolean sampleLeft = false;
        boolean sampleRight = false;
        boolean sampleCenter = false;
        telemetry.addData("Gold Position", goldPosition);
        telemetry.update();

        robot.StopAll();


        runtime.reset();
        robot.DriveTimed(DriveDirection.Forward, 200);

        runtime.reset();


        if (goldPosition == MineralDetector.MineralPosition.Left) {
            robot.DriveTimed(DriveDirection.Left, 385);
            robot.DriveTimed(DriveDirection.Forward, 800);
            sampleLeft = true;
        } else if (goldPosition == MineralDetector.MineralPosition.Right) {
            robot.DriveTimed(DriveDirection.Right, 380);
            robot.DriveTimed(DriveDirection.Forward, 900);
            sampleRight = true;
        } else if (goldPosition == MineralDetector.MineralPosition.Center) //gold center
        {
            robot.DriveTimed(DriveDirection.Forward, 600);
            sampleCenter = true;
        } else {
            robot.DriveTimed(DriveDirection.Forward, 900);
            sampleCenter = true;
        }
        if (sampleLeft == true) {
            robot.DriveTimed(DriveDirection.Left, 530);
            robot.ArmJointDrop();
            robot.DriveTimed(DriveDirection.Forward, 1300);
            while (opModeIsActive() && robot.armJointMotor.getCurrentPosition() > 1070) {
                robot.ArmJointDrop();
            }
            robot.ArmJointStop();
            robot.DriveTimed(DriveDirection.Forward, 800);
            robot.armReleaseServo.setPwmDisable();
            robot.armCombineMotor.setPower(.75);
            sleep(500);
            robot.armCombineMotor.setPower(0);
            robot.DriveTimed(DriveDirection.Backward, 2400);
        } else if (sampleRight == true) {
            robot.DriveTimed(DriveDirection.Backward, 800);
            robot.DriveTimed(DriveDirection.Left, 900);
            robot.DriveForwardCheckObstruction(1400);
            robot.DriveTimed(DriveDirection.Left, 470);
            robot.ArmJointDrop();
            robot.DriveTimed(DriveDirection.Forward, 1170);
            while (opModeIsActive() && robot.armJointMotor.getCurrentPosition() > 1070) {
                robot.ArmJointDrop();
            }
            robot.ArmJointStop();
            robot.armReleaseServo.setPwmDisable();
            robot.armCombineMotor.setPower(.75);
            sleep(500);
            robot.armCombineMotor.setPower(0);
            robot.DriveTimed(DriveDirection.Backward, 2100);
        } else if (sampleCenter == true) {
            robot.DriveTimed(DriveDirection.Backward, 750);
            robot.DriveTimed(DriveDirection.Left, 650);
            robot.DriveTimed(DriveDirection.Forward, 500);
            robot.DriveForwardCheckObstruction(900);
            robot.DriveTimed(DriveDirection.Left, 400);
            robot.ArmJointDrop();
            robot.DriveTimed(DriveDirection.Forward, 1250);
            while (opModeIsActive() && robot.armJointMotor.getCurrentPosition() > 1070) {
                robot.ArmJointDrop();
            }
            robot.ArmJointStop();
            robot.armReleaseServo.setPwmDisable();
            robot.armCombineMotor.setPower(.75);
            sleep(800);
            robot.armCombineMotor.setPower(0);
            robot.DriveTimed(DriveDirection.Backward, 2550);
        } else {
            robot.DriveTimed(DriveDirection.Backward, 750);
            robot.DriveTimed(DriveDirection.Left, 620);
            robot.DriveForwardCheckObstruction(1400);
            robot.DriveTimed(DriveDirection.Left, 460);
            robot.ArmJointDrop();
            robot.DriveTimed(DriveDirection.Forward, 1250);
            while (opModeIsActive() && robot.armJointMotor.getCurrentPosition() > 1200) {
                robot.ArmJointDrop();
            }
            robot.ArmJointStop();
            robot.armReleaseServo.setPwmDisable();
            robot.armCombineMotor.setPower(.75);
            sleep(800);
            robot.armCombineMotor.setPower(0);
            robot.DriveTimed(DriveDirection.Backward, 2400);
        }


        robot.StopAll();
        mineralDetector.Shutdown();
    }
}
