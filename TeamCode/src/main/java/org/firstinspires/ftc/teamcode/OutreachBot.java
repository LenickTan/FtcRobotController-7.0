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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
@TeleOp(name="OutreachBot", group="Iterative Opmode")
public class OutreachBot extends OpMode


{
    boolean changed = false; //for the claw
    double shooterPower = 0;    //power for shooter
    double pulleyPower = 0;     //power for pulley
    double intakePower = 0;     //power for intake

    //motors------------------------------------v
    public DcMotor leftFront;   //drivetrainwheels
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor shooter;     //wheels to shoot rings
    public DcMotor shooter2;

    public DcMotor pulley;      //pulley to lift for shooter

    public DcMotor intake; //power intake

    //servos----------------------------------
    public Servo pinwheel1;       //right intake to straighten ring
    public Servo pinwheel2;       //left intake to straighten ring

    public Servo lift1;  //wobble goal lifter
    public Servo lift2;
    public Servo lift3;

    public Servo claw; //wobble goal clasp/claw

    public Servo flicker; //pushes ring to shooter

    @Override
    public void init() {

//SHOOTER
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter2 = hardwareMap.dcMotor.get("shooter2");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);


        pulley = hardwareMap.dcMotor.get("pulley");
//DRIVETRAIN
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

//WOBBLE GOAL
        lift1 = hardwareMap.servo.get("lift1");
        lift2= hardwareMap.servo.get("lift2");
        lift3= hardwareMap.servo.get("lift3");
        claw = hardwareMap.servo.get("claw");

//INTAKE
        intake = hardwareMap.dcMotor.get("intake");
        flicker = hardwareMap.servo.get("flicker");
        pinwheel1 = hardwareMap.servo.get("pinwheel1");
        pinwheel2 = hardwareMap.servo.get("pinwheel2");
    }

    @Override
    public void loop() {
        float x;
        float y;
        float z;

        //GAMEPAD 2 ------------------------------------------
        //drivetrain
        if (Math.abs(gamepad2.right_stick_x) > .1) {
            x = gamepad2.right_stick_x;
        } else {
            x = 0;
        }

        if (Math.abs(gamepad2.left_stick_y) > .1) {
            y = gamepad2.left_stick_y;
        } else {
            y = 0;
        }

        if (Math.abs(gamepad2.left_stick_x) > .1) {
            z = gamepad2.left_stick_x;
        } else {
            z = 0;
        }

         /*   pulley lift
        left trigger = goes up
        right trigger = goes down */

        if (Math.abs(gamepad2.left_trigger) > .1) {
            pulleyPower = -1;

        } else if (Math.abs(gamepad2.right_trigger) > .1) {
            pulleyPower = 1;

        } else {
            pulleyPower = 0;

        }

        //shooter
        if (gamepad2.x) {
            shooterPower = 0.7;

        } else if (gamepad2.y){
            shooterPower = 0;

        }

        if(gamepad2.a){
            shooterPower = 1;
        }

        //GAMEPAD 1 ------------------------------------------

        //INTAKE--------------------------------------------
        //intake pinwheels (moves in opposite directions)
        if((gamepad1.right_trigger) >0.1) {
            pinwheel2.setPosition(pinwheel2.getPosition() + 0.1);
            pinwheel1.setPosition(pinwheel1.getPosition() - 0.1);

        } else if ((gamepad1.left_trigger) >0.1) {
            pinwheel2.setPosition(pinwheel2.getPosition() - 0.1);
            pinwheel1.setPosition(pinwheel1.getPosition() + 0.1);

        } else {
            pinwheel2.setPosition(0.5);
            pinwheel1.setPosition(0.5);
        }
        //powers intake
        if (Math.abs(gamepad1.left_trigger) > .1) {
            intakePower = -1;

        } else if (Math.abs(gamepad1.right_trigger) > .1) {
            intakePower = 1;

        } else {
            intakePower = 0;
        }

        //flicker moves back and forth 90 degrees
        if ((gamepad1.right_bumper) ) {
            flicker.setPosition(flicker.getPosition() + 0.1);
        } else if ((gamepad1.left_bumper) ) {
            flicker.setPosition(flicker.getPosition() - 0.1);
        } else {
            flicker.setPosition(0.5);
        }

        //WOBBLE GOAL-------------------------------
        if (gamepad1.a) {
            lift1.setPosition(1);
            lift2.setPosition(1);
            lift3.setPosition(1);
        }


        if (gamepad1.b) {

            if (gamepad1.x) {
                lift1.setPosition(0.75);
                lift2.setPosition(0.55);
                lift3.setPosition(0.65);
            }

            //open close claw
            if (gamepad1.y && !changed) {
                if(claw.getPosition() == 0 )
                    claw.setPosition(0.99);
                else {
                    claw.setPosition(0);
                }
                changed = true;
            } else if (!gamepad1.y) {
                changed = false;
            lift1.setPosition(0);
            lift2.setPosition(0);
            lift3.setPosition(0);
        }
        }

//-------------------------------------------------------------
        shooter.setPower(shooterPower);
        shooter2.setPower(shooterPower);

        pulley.setPower(pulleyPower*0.15);

        intake.setPower(intakePower);

        leftBack.setPower((y - x + z)*.75);
        leftFront.setPower((y + x - z)*.75);

        rightBack.setPower((y - x - z)*.75);
        rightFront.setPower((y + x + z)*.75);

        telemetry.addData ("rightfrpnt",
                rightFront.getCurrentPosition());
        telemetry.addData ("rightback",
                rightBack.getCurrentPosition());
        telemetry.addData ("leftback",
                leftBack.getCurrentPosition());
        telemetry.addData ("leftfrpnt",
                leftFront.getCurrentPosition());
    }
}