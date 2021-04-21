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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="Basic: Linear OpMode", group="Linear Opmode")
//@Disabled
public class Teleop extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Code Version = Golden Master");
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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD); //Can be changed based on motor configuration
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE); //Can be changed based on motor configuration
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); //Can be changed based on motor configuration
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //Can be changed based on motor configuration
        ServoLeftRoller.setDirection(CRServo.Direction.REVERSE);
        //ServoRightRoller.setDirection(CRServo.Direction.REVERSE);
        Elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftShooter.setDirection(DcMotor.Direction.FORWARD); //Can be changed based on motor configuration
        RightShooter.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftBackPower;
            double rightBackPower;
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPowerRight;
            double rightBackPowerRight;
            double leftFrontPowerRight;
            double rightFrontPowerRight;
            double ServoRightRollerPower;
            double ServoLeftRollerPower;
            double MotorLeftShooterPower;
            double MotorRightShooterPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double LeftForward = gamepad1.left_stick_y;
            double LeftSide  =  -gamepad1.left_stick_x;
            double RightSide  =  gamepad1.right_stick_x;
            boolean Shoot = gamepad1.right_bumper;
            boolean Collect = gamepad1.b;
            boolean ElevatorControl = gamepad1.a;
            int CollectasInt;
            double ShootasInt;
            int ElevatorasInt;
            if(Collect == true)
            {
                CollectasInt = 1;
            } else
            {
                CollectasInt = 0;
            }
            if(Shoot == true)
            {
                ShootasInt = 1;
            } else
            {
                ShootasInt = 0;
            }
            if(ElevatorControl == true)
            {
                ElevatorasInt = 1;
            } else
            {
                ElevatorasInt = 0;
            }
            //This is really cringe but I dont want to do math

                //Left Stick
                leftBackPower = Range.clip(LeftForward - LeftSide + RightSide, -1.0, 1.0);
                rightBackPower = Range.clip(LeftForward + LeftSide - RightSide, -1.0, 1.0);
                leftFrontPower = Range.clip(LeftForward + LeftSide + RightSide, -1.0, 1.0);
                rightFrontPower = Range.clip(LeftForward - LeftSide - RightSide, -1.0, 1.0);

                //Right stick
                /*leftBackPowerRight = Range.clip(RightSide, -1.0, 1.0);
                rightBackPowerRight = Range.clip(-RightSide, -1.0, 1.0);
                leftFrontPowerRight = Range.clip(-RightSide, -1.0, 1.0);
                rightFrontPowerRight = Range.clip(RightSide, -1.0, 1.0);*/

            ServoRightRollerPower  = CollectasInt;
            ServoLeftRollerPower = CollectasInt;
            // Send calculated power to wheels
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            ServoRightRoller.setPower(ServoRightRollerPower);
            ServoLeftRoller.setPower(ServoLeftRollerPower);
            Elevator.setPower(ElevatorasInt);
            LeftShooter.setPower(ShootasInt);
            RightShooter.setPower(ShootasInt);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("MotorsNEw", "Frontleft (%.2f), Frontright (%.2f), Backright (%.2f), Backleft (%.2f)", leftFrontPower, rightFrontPower, rightBackPower, leftBackPower);
            telemetry.addData("Servos", "" + Collect);
            telemetry.update();
        }
    }
}
