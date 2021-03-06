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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;


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

@TeleOp(name="Auto", group="Linear Opmode")
//@Disabled
public class Auto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor LeftShooter = null;
    private DcMotor RightShooter = null;
    private DcMotor Elevator = null;
    private CRServo ServoLeftRoller = null;
    private CRServo ServoRightRoller = null;
    private ColorSensor ColorSenser = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Code Version = Auto");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        LeftShooter = hardwareMap.get(DcMotor.class, "LeftShooter");
        RightShooter = hardwareMap.get(DcMotor.class, "RightShooter");
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        ServoLeftRoller = hardwareMap.get(CRServo.class, "ServoLeftRoller");
        ServoRightRoller = hardwareMap.get(CRServo.class, "ServoRightRoller");
        ColorSenser = hardwareMap.colorSensor.get("Color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); //Can be changed based on motor configuration
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); //Can be changed based on motor configuration
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //Can be changed based on motor configuration
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //Can be changed based on motor configuration
        //ServoLeftRoller.setDirection(CRServo.Direction.FORWARD);
        //ServoRightRoller.setDirection(CRServo.Direction.REVERSE);
        Elevator.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftShooter.setDirection(DcMotor.Direction.REVERSE); //Can be changed based on motor configuration
        RightShooter.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            telemetry.addData("Red: ", ColorSenser.red());
            telemetry.addData("Green: ", ColorSenser.green());
            telemetry.addData("Blue: ", ColorSenser.blue());
            telemetry.update();
        }
       /* leftBackDrive.setPower(.5);
        rightBackDrive.setPower(.5);
        rightFrontDrive.setPower(.5);
        leftFrontDrive.setPower(.5);
        sleep(6500);
        leftBackDrive.setPower(-.5);
        rightBackDrive.setPower(-.5);
        rightFrontDrive.setPower(-.5);
        leftFrontDrive.setPower(-.5);
        sleep(1250);*/

    }
}
