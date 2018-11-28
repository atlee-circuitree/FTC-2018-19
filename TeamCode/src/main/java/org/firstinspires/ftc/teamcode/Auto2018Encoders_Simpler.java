

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

@Autonomous(name = "Autonomous2018Encoders_Simpler", group = "Linear Opmode")

public class Auto2018Encoders_Simpler extends LinearOpMode {


    // Declare OpMode members.
    hardware2018 robot = new hardware2018();
    MineralDetector mineralDetector = new MineralDetector();
    private ElapsedTime runtime = new ElapsedTime();
    Drop18 drop18 = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(this, hardwareMap);
        robot.ResetEncoders();

        mineralDetector.init(hardwareMap, telemetry);

        drop18 = new Drop18(robot, mineralDetector, telemetry, this);

        robot.armCombineServo.setPosition(0.5);

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

        drop18.dropBot();
	//end of dropping 
	
        runtime.reset();
        robot.DriveTimed(DriveDirection.Forward, 200);


        if (goldPosition == MineralDetector.MineralPosition.Left) {
            robot.DriveTimed(DriveDirection.Left, 450);
            robot.DriveTimed(DriveDirection.Forward, 800);
        } else if (goldPosition == MineralDetector.MineralPosition.Right) {
            robot.DriveTimed(DriveDirection.Right, 450);
            robot.DriveTimed(DriveDirection.Forward, 800);
        } else if (goldPosition == MineralDetector.MineralPosition.Center) //gold center
        {
            robot.DriveTimed(DriveDirection.Forward, 800);
        }

        robot.StopAll();
        mineralDetector.Shutdown();
    }
}
