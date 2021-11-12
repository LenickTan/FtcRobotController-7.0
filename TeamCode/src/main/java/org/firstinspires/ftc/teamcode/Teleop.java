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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Teleop", group="Iterative Opmode")
public class Teleop extends OpMode{

    double leftDrivePower = 0;
    double rightDrivePower = 0;
    double pulleyPower = 0;
    double top = 0;
   // boolean changed = false;


    public DcMotor leftSide1;       // wheels
    public DcMotor leftSide2;
    public DcMotor rightSide1;
    public DcMotor rightSide2;
    public DcMotor topwheel;
    public DcMotor arm;
    public Servo intake1;
    public Servo intake2;


    @Override
    public void init() {
        // initializing motors
        leftSide1 = hardwareMap.dcMotor.get("leftSide1");
        leftSide2 = hardwareMap.dcMotor.get("leftSide2");
        rightSide1 = hardwareMap.dcMotor.get("rightSide1");
        rightSide2 = hardwareMap.dcMotor.get("rightSide2");
        rightSide1.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSide2.setDirection(DcMotorSimple.Direction.REVERSE);

        intake1 = hardwareMap.servo.get("intake1");
        intake2 = hardwareMap.servo.get("intake2");


        topwheel = hardwareMap.dcMotor.get("topwheel");
        arm = hardwareMap.dcMotor.get("arm");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

    }

    @Override
    public void loop() {
        //Gamepad 1 Controls

        //drivetrian

        leftDrivePower = gamepad1.left_stick_y + gamepad1.left_stick_x;
        rightDrivePower = gamepad1.left_stick_y - gamepad1.left_stick_x;

//topwheel

        if (Math.abs(gamepad1.left_trigger) > .1) {
            top = -1;

        } else if (Math.abs(gamepad2.right_trigger) > .1) {
            top = 1;

        } else {
            top = 0;

        }

        //gamepad 2
      /*  if (gamepad2.y && !changed) {
            if (intake2.getPosition() == 0)
                intake2.setPosition(1);
            else {
                intake2.setPosition(0);
            }

            if (gamepad2.y && !changed) {
                if (intake1.getPosition() == 0)
                    intake1.setPosition(1);
                else {
                    intake1.setPosition(0);
                }

            }


       */
//claw
        if (gamepad2.a) {
            intake1.setPosition(1);
            intake2.setPosition(1);
        }


        if (gamepad2.b) {

            if (gamepad1.x) {
                intake1.setPosition(0);
                intake2.setPosition(0);
            }
//arm
            if (Math.abs(gamepad2.left_trigger) > .1) {
                pulleyPower = -1;

            } else if (Math.abs(gamepad2.right_trigger) > .1) {
                pulleyPower = 1;

            } else {
                pulleyPower = 0;

            }

            // initializing powers
            leftSide1.setPower(leftDrivePower);
            leftSide2.setPower(leftDrivePower);
            rightSide1.setPower(rightDrivePower);
            rightSide2.setPower(rightDrivePower);

            arm.setPower(pulleyPower*0.5);




        }



    }

}


