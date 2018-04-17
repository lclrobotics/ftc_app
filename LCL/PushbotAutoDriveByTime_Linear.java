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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file illustrates the concept of driving a path based on time.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backwards for 1 Second
 *   - Stop and close the claw.
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Pushbot: Range Sensor Drive", group="Pushbot")
@Disabled
public class PushbotAutoDriveByTime_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     FIELD_LENGTH  = 365.76;
    static double           X_POS         = 0.0;
    static double           Y_POS         = 0.0;



    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        ModernRoboticsI2cRangeSensor rangeRight= hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_right");
        ModernRoboticsI2cRangeSensor rangeFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_front");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        ColorSensor baseColor   = hardwareMap.get(ColorSensor.class, "base_sensor");
        Servo colorArm = hardwareMap.get(Servo.class, "color_arm");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //grab block
        robot.leftClaw.setPosition(0.5);
        robot.rightClaw.setPosition(-0.5);
        colorSensor.enableLed(true);
        colorArm.setPosition(0.0);
        sleep(2000);
        //knock over ball and drive back
        if(
                ((baseColor.blue() > baseColor.red()) && (colorSensor.blue() > colorSensor.red()))
                || ((baseColor.red() > baseColor.blue()) && (colorSensor.red() > colorSensor.blue()))
                //evaluates if ball and base are same color for any position
                ){
            robot.leftDrive.setPower(FORWARD_SPEED);
            robot.rightDrive.setPower(FORWARD_SPEED);
        }
        else{
            robot.leftDrive.setPower(-FORWARD_SPEED);
            robot.rightDrive.setPower(-FORWARD_SPEED);
        }
        sleep(200);
        //reset to defaults
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        colorArm.setPosition(0.0);
        //drive forward to clear platform
        robot.leftDrive.setPower(FORWARD_SPEED);
        robot.rightDrive.setPower(FORWARD_SPEED);
        sleep(500);
        robot.leftDrive.setPower(0.0);
        robot.rightDrive.setPower(0.0);
        //figure out position
        String robotPos = null;
        X_POS = rangeRight.getDistance(DistanceUnit.CM);
        Y_POS = rangeFront.getDistance(DistanceUnit.CM);
        if(baseColor.blue() > baseColor.red()){//if on blue side
            if(X_POS > 1000){
                robotPos = "blue_right";
            }
            else{
                robotPos = "blue_left";
            }
        }
        else{
            if(X_POS > 1000){
                robotPos = "red_left";
            }
            else{
                robotPos = "red_right";
            }
        }

    }
}
