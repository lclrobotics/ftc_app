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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwarePushbot;
//C:\Users\lcl\Downloads\ftc_app-mastertest\ftc_app-master\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\HardwarePushbot.java

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TBirds:Controller Mode", group="Pushbot")
//prev. name: PushbotTeleopTank_Iterative_TBirds_ControllerMode
public class TBirds_DriverController_OpMode extends OpMode{

    /* Declare OpMode members. */
    org.firstinspires.ftc.teamcode.HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          topClawOffset  = 0.0 ;                  // Servo mid position
    double          botClawOffset  = 0.0 ;
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    double          grabber_flipOffset = 0.0;
    double          grabber_clawOffset = 0.0;

    public DcMotor side_to_side_extendy_arm_thing = null;
    //public Servo grabber_flip = null;
    //public Servo grabber_claw = null;
    public Servo colorArm = null;
    public Servo topLeftClaw = null;
    public Servo topRightClaw = null;
    public Servo botLeftClaw = null;
    public Servo botRightClaw = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()  {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        side_to_side_extendy_arm_thing = hardwareMap.get(DcMotor.class, "extend");

        topRightClaw = robot.rightClaw;
        topLeftClaw = robot.leftClaw;
        botRightClaw = hardwareMap.get(Servo.class, "bot_right_claw");
        botLeftClaw = hardwareMap.get(Servo.class, "bot_left_claw");

        //grabber_flip = hardwareMap.get(Servo.class, "grabber_flip");
        //grabber_claw = hardwareMap.get(Servo.class, "grabber_claw");
        colorArm = hardwareMap.get(Servo.class, "color_arm");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        colorArm.setPosition(1.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);


        // Use gamepad left & right Bumpers to open and close the claw

        if (gamepad2.right_bumper)
            topClawOffset +=  CLAW_SPEED;
        else if (gamepad2.left_bumper)
            topClawOffset -= CLAW_SPEED;
        // Move both servos to new position.  Assume servos are mirror image of each other.
        topClawOffset = Range.clip(topClawOffset, -0.5, 0.5);
        topLeftClaw.setPosition(robot.MID_SERVO + topClawOffset);
        topRightClaw.setPosition(robot.MID_SERVO - topClawOffset);

        if(gamepad2.x){//(gamepad2.right_trigger >= 0.9)){
            botClawOffset += CLAW_SPEED;
        }
        else if(gamepad2.b){//(gamepad2.left_trigger >= 0.9)){
            botClawOffset -= CLAW_SPEED;
        }
        botClawOffset = Range.clip(botClawOffset, -0.5, 0.5);
        botLeftClaw.setPosition(robot.MID_SERVO + botClawOffset);
        botRightClaw.setPosition(robot.MID_SERVO - botClawOffset);


        //use arrows to contrl arm claw
        /*
        if (gamepad1.dpad_up || gamepad2.dpad_up){
            grabber_flipOffset += CLAW_SPEED;
        }
        else if (gamepad1.dpad_down || gamepad2.dpad_down){
            grabber_flipOffset -= CLAW_SPEED;
        }
        else{
            grabber_flipOffset = 0.0;
        }
        grabber_flipOffset = Range.clip(grabber_flipOffset, -1.5  , 1.5);
        grabber_flip.setPosition(robot.MID_SERVO + grabber_flipOffset);

        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            grabber_clawOffset -= CLAW_SPEED;
        }
        else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            grabber_clawOffset += CLAW_SPEED;
        }
        grabber_clawOffset = Range.clip(grabber_clawOffset, -1.5, 1.5);
        grabber_claw.setPosition(robot.MID_SERVO + grabber_clawOffset);*/

        //same thing as other servs for idol grabber




        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad2.y){
            robot.leftArm.setPower(robot.ARM_UP_POWER);}
        else if (gamepad2.a){
            telemetry.addData("test", "gamepad2.a");
            robot.leftArm.setPower(robot.ARM_DOWN_POWER);}
        else{
            robot.leftArm.setPower(0.0);
            telemetry.addData("stop", "is on");}
        //move idol arm using (X) and (B)
        /*if (gamepad2.x){
            side_to_side_extendy_arm_thing.setPower(0.1);
        }
        else if(gamepad2.b){
            side_to_side_extendy_arm_thing.setPower(-0.1);
        }
        else{
            side_to_side_extendy_arm_thing.setPower(0.0);
        }*/

        // Send telemetry message to signify robot running;
        telemetry.addData("armPower", "%.2f", robot.leftArm.getPower());
        telemetry.addData("top claw",  "Offset = %.2f", topClawOffset);
        telemetry.addData("bottom claw", "Offset = %.2f", botClawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("triggers", "R:%.2f L: %.2f", gamepad1.right_trigger, gamepad1.left_trigger);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}