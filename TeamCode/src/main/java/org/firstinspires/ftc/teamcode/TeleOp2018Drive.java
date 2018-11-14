

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

@TeleOp(name = "TeleOp2018Manual", group = "Linear Opmode")

public class TeleOp2018Drive extends LinearOpMode {


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

	//create a timerVarible
	
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;
        double combineSpeed = 0;
        double armCombineOpenEndTime = 0;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1, 1);
            rightPower = Range.clip(drive - turn, -1, 1);

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            robot.leftDriveFront.setPower(leftPower);
            robot.leftDriveRear.setPower(leftPower);
            robot.rightDriveFront.setPower(rightPower);
            robot.rightDriveRear.setPower(rightPower);

            robot.armCombineMotor.setPower(combineSpeed);

            //get data for combine speed
            //starts off, stays on in whatever the last direction was
            if (gamepad2.left_bumper) {
                combineSpeed = 1;
            } else if (gamepad2.right_bumper) {
                combineSpeed = -1;
            } else if (gamepad2.right_trigger > .85){
                combineSpeed = 0;
            }


            //move arm in and out
            if (gamepad2.left_stick_y > 0.5)  //stick values are -1 to 1 - so we have to set a "threshold" that triggers movement
                robot.armExtendMotor.setPower(-1);
            else if (gamepad2.left_stick_y < -0.5)
                robot.armExtendMotor.setPower(1);
            else
                robot.armExtendMotor.setPower(0);

            //move arm up and down
            if (gamepad2.right_stick_y > 0.5)  //stick values are -1 to 1 - so we have to set a "threshold" that triggers movement
                robot.armJointMotor.setPower(-1);
            else if (gamepad2.right_stick_y < -0.5)
                robot.armJointMotor.setPower(1);
            else
                robot.armJointMotor.setPower(0);

            //open and close the combine's servo to hold/release game pieces - option 1 - open and close triggered by buttons
            //if (gamepad2.left_trigger > 0)
            //    robot.armCombineServo.setPosition(0);
            //else if (gamepad2.right_trigger > 0)
            //    robot.armCombineServo.setPosition(0.5);

            //open and close the combine's servo to hold/release game pieces - option 2 - open triggered by a button, close after a specific timeframe
            //first time the button is pushed - set a timestamp for when the servo should go back into position
            if (gamepad2.a && armCombineOpenEndTime == 0) {
                armCombineOpenEndTime = runtime.milliseconds() + 2000; //right now + 2 seconds.  We use milliseconds instead of seconds to avoid rounding problems
                robot.armCombineServo.setPosition(1);
            }
            if (armCombineOpenEndTime < runtime.milliseconds()) //have our 2 seconds passed?
            {
                armCombineOpenEndTime = 0;
                robot.armCombineServo.setPosition(0.7);
            }

            //drive the climbing/drop motor
            if (gamepad1.y)
                robot.climbMotor.setPower(1);
            else if (gamepad1.a)
                robot.climbMotor.setPower(-1);
            else
                robot.climbMotor.setPower(0);

            //trigger the arm release servo - can be used in-game in case of emergency,
            //or to manually go through the motions of start-up without autonomous code
            if (gamepad1.x) {
                robot.armReleaseServo.setPwmEnable();
                robot.armReleaseServo.setPosition(-.7);
            } else {
                //robot.armReleaseServo.setPosition(1);
                robot.armReleaseServo.setPwmDisable();

            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
            //show power of wheles
            telemetry.addData("E-ExtendMotor", robot.armExtendMotor.getCurrentPosition());
            telemetry.addData("E-armJointMotor", robot.armJointMotor.getCurrentPosition());
            telemetry.addData("E-armCombineMotor", robot.armCombineMotor.getCurrentPosition());
            telemetry.addData("E-leftdrivemotor", robot.leftDriveFront.getCurrentPosition());
            telemetry.addData("E-leftDrivemotorRear", robot.leftDriveRear.getCurrentPosition());
            telemetry.addData("E-rightDrivemotor", robot.rightDriveFront.getCurrentPosition());
            telemetry.addData("E-rightDriveMotorRear", robot.rightDriveRear.getCurrentPosition());

            telemetry.addData("E-climbMotor", robot.climbMotor.getCurrentPosition());

        }
    }
}
