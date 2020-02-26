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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.internal.android.dx.dex.code.VariableSizeInsn;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleopBaymaxx", group="Teleop")

public class TeleopTrial3 extends LinearOpMode {

    HardwareBaymaxx robot = new HardwareBaymaxx();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        telemetry.update();

        robot.BleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FleftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.BrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.FrightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftwheel.setDirection(DcMotor.Direction.REVERSE);
        robot.rightwheel.setDirection(DcMotor.Direction.FORWARD);

        //robot.clamp.setPosition(1);
        robot.push.setPosition(0.18);
        //robot.turn.setPosition(0.2);

        double speedAdjust = 7;
        double leftPower = 0;

        waitForStart();
        runtime.reset();

        telemetry.addData("Before Servo Position ", robot.push.getPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();

        while (opModeIsActive()) {

            if (gamepad1.dpad_left == true) {
                speedAdjust -= 1;
            }
            if (gamepad1.dpad_right ==true) {
                 speedAdjust += 1;
            }

            //The amount of power in each wheel, set by the left and right sticks on the controller
            robot.BleftDrive.setPower((gamepad1.left_stick_y +  gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.BrightDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FleftDrive.setPower((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (-speedAdjust / 10));
            robot.FrightDrive.setPower((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (-speedAdjust / 10));

            //Intake Wheels Eject
            if (gamepad2.a) {
                robot.leftwheel.setDirection(DcMotor.Direction.REVERSE);
                robot.rightwheel.setDirection(DcMotor.Direction.FORWARD);
                robot.rightwheel.setPower(2);
                robot.leftwheel.setPower(2);
            } else{
                robot.rightwheel.setPower(0);
                robot.leftwheel.setPower(0);
            }

            //Intake Wheels Intake
            if (gamepad2.x) {
                robot.leftwheel.setDirection(DcMotor.Direction.FORWARD);
                robot.rightwheel.setDirection(DcMotor.Direction.REVERSE);
                robot.rightwheel.setPower(2);
                robot.leftwheel.setPower(2);
            } else {
                robot.rightwheel.setPower(0);
                robot.leftwheel.setPower(0);
            }

            //lifting mechanism
            double drive = gamepad2.left_stick_y;
            leftPower    = Range.clip(drive, -1.0, 1.0) ;
            robot.leftlift.setPower(leftPower);
            robot.rightlift.setPower(leftPower);

            //Clamp close
           /* if (gamepad1.a) {
                robot.clamp.setPosition(0); //put the servo to holding on to the block
            } else {
                robot.clamp.setPosition(1);
            }*/

            //moving block
            if (gamepad2.right_bumper) {
                robot.backward.setPosition(0.6);
                //sleep(500);
                robot.forward.setPosition(0.6);

            } else {
                robot.backward.setPosition(0.51);
                robot.forward.setPosition(0.50);
            }

            if (gamepad2.left_bumper) {
                robot.forward.setPosition(0.4);
                //sleep(500);
                robot.backward.setPosition(0.4);
            } else{
                robot.forward.setPosition(0.50);
                robot.backward.setPosition(0.51);
            }

            //Turn out of Robot
           /* if (gamepad1.x) {
                robot.turn.setPosition(1);
            }
            else{
                robot.turn.setPosition(0);
            }
*/
            //Push down
            if (gamepad1.left_bumper) {
                robot.push.setPosition(0.18);
            }

            //All the way up
            if (gamepad1.right_bumper) {
                robot.push.setPosition(1);
            }

            telemetry.addData("Servo Position ", robot.push.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}