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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Drives arm motors with a controller for testing and positioning the robot for measuring
 */

@TeleOp(name = "Arm Motor Drive", group = "Linear Opmode")

public class ArmMotorDrive extends LinearOpMode {


    // Declare OpMode members.
    hardware2018 robot = new hardware2018();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        robot.armCombineServo.setPosition(0.5);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPowerF;
            double leftPowerR;
            double rightPowerF;
            double rightPowerR;

            double combineSpeed = 0;
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPowerF = Range.clip(drive + turn, -0.7, 0.7);
            leftPowerR = Range.clip(drive + turn, -0.7, 0.7);
            rightPowerF = Range.clip(drive - turn, -0.7, 0.7);
            rightPowerR = Range.clip(drive - turn, -0.7, 0.7);


            //get data for combine speed
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                combineSpeed = 0;
            } else if (gamepad1.right_bumper) {
                combineSpeed = .9;
            } else if (gamepad1.left_bumper) {
                combineSpeed = -.9;
            }

            //move arm in and out
            if(gamepad1.dpad_down)
                robot.armExtendMotor.setPower(1);
            else if(gamepad1.dpad_up)
                robot.armExtendMotor.setPower(-1);
            else
                robot.armExtendMotor.setPower(0);

            //move arm up and down
            if(gamepad1.dpad_left)
                robot.armJointMotor.setPower(1);
            else if(gamepad1.dpad_right)
                robot.armJointMotor.setPower(-1);
            else
                robot.armJointMotor.setPower(0);

            if(gamepad1.left_trigger > 0)
                robot.armCombineServo.setPosition(0);
            else if(gamepad1.right_trigger > 0)
                robot.armCombineServo.setPosition(0.5);

            if(gamepad1.y)
                robot.climbMotor.setPower(1);
            else if (gamepad1.a)
                robot.climbMotor.setPower(-1);
            else
                robot.climbMotor.setPower(0);

            if(gamepad1.x)
            {
                robot.armReleaseServo.setPwmEnable();
                robot.armReleaseServo.setPosition(0);
            }
            else
            {
                robot.armReleaseServo.setPwmDisable();
                //robot.armReleaseServo.setPosition(1);
            }

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.leftDriveFront.setPower(leftPowerF);
            robot.leftDriveRear.setPower(leftPowerR);
            robot.rightDriveFront.setPower(rightPowerF);
            robot.rightDriveRear.setPower(rightPowerR);

            robot.armCombineMotor.setPower(combineSpeed);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
