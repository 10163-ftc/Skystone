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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have en coders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Straight", group="Pushbot")
public class AutoStraight extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareBaymaxx         robot   = new HardwareBaymaxx();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.FleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.FrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          robot.FrightDrive.getCurrentPosition(),
                          robot.FleftDrive.getCurrentPosition(),
                          robot.BleftDrive.getCurrentPosition(),
                          robot.BrightDrive.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //strafe to left
        encoderDrive(DRIVE_SPEED,  5,  5, 5,5,3);

        //stop
        robot.FrightDrive.setPower(0);
        robot.FleftDrive.setPower(0);
        robot.BrightDrive.setPower(0);
        robot.BleftDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void encoderDrive(double speed,
                             double FleftInches, double FrightInches, double BleftInches,double BrightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.FleftDrive.getCurrentPosition() + (int) (FleftInches * COUNTS_PER_INCH);
            newLeftTarget = robot.BleftDrive.getCurrentPosition() + (int) (BleftInches * COUNTS_PER_INCH);
            newRightTarget = robot.FrightDrive.getCurrentPosition() + (int) (FrightInches * COUNTS_PER_INCH);
            newRightTarget = robot.BrightDrive.getCurrentPosition() + (int) (BrightInches * COUNTS_PER_INCH);

            robot.FleftDrive.setTargetPosition(newLeftTarget);
            robot.BleftDrive.setTargetPosition(newLeftTarget);
            robot.FrightDrive.setTargetPosition(newRightTarget);
            robot.BrightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.FleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FleftDrive.setPower(Math.abs(speed));
            robot.BrightDrive.setPower(Math.abs(speed));
            robot.FrightDrive.setPower(Math.abs(speed));
            robot.BleftDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.FleftDrive.isBusy() && robot.FrightDrive.isBusy() && robot.BrightDrive.isBusy() && robot.BleftDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.FleftDrive.getCurrentPosition(),
                        robot.FrightDrive.getCurrentPosition(),
                        robot.BleftDrive.getCurrentPosition(),
                        robot.BrightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.FleftDrive.setPower(0);
            robot.BleftDrive.setPower(0);
            robot.FrightDrive.setPower(0);
            robot.BrightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}