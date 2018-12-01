
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

// code based on BergTest throught the copy/paste paradime (Im saying dont use this)

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "halfStopAuto", group = "Linear Opmode")
@Disabled

public class halfStopAuto extends LinearOpMode {


    // Declare OpMode members.
    hardware2018 robot = new hardware2018();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(this, hardwareMap);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //drop the bot
        // sleep times are to run motors without jumping to the next ones or using giant while loops



        //unfurl the robot arm
        robot.armReleaseServo.setPwmEnable();
        robot.armReleaseServo.setPosition(-1);
        robot.climbMotor.setPower(1);
        sleep(500);
        robot.armReleaseServo.setPosition(0);
        //robot.climbMotor.setPower(0);
        sleep(2000);
        robot.armJointMotor.setPower(1);
        sleep(2000);
        robot.armJointMotor.setPower(0);
        sleep(500);
        robot.armExtendMotor.setPower(1);
        sleep(2000);
        robot.armExtendMotor.setPower(0);

        sleep(1500);
        //needs to have ~8500 ms before it
        robot.climbMotor.setPower(0);


        // find current rotatedness of wheels so the robot can do ~2.23 revs
        // note: our drive motors have 7 ticks per rotation
        int originPos = robot.rightDriveFront.getCurrentPosition();
        int revolveNum = originPos + 900;

        while (robot.rightDriveFront.getCurrentPosition() < revolveNum)
        {
            robot.leftDriveFront.setPower(1);
            robot.leftDriveRear.setPower(1);
            robot.rightDriveFront.setPower(1);
            robot.rightDriveRear.setPower(1);
        }
        
            robot.leftDriveFront.setPower(0);
            robot.leftDriveRear.setPower(0);
            robot.rightDriveFront.setPower(0);
            robot.rightDriveRear.setPower(0);
            robot.climbMotor.setPower(0);
            robot.armJointMotor.setPower(0);
            robot.armExtendMotor.setPower(0);
            robot.armCombineMotor.setPower(0);



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();

        }
    }
}
