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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="RCXD Motor Test", group="Linear Opmode")
//@Disabled
public class RCXDMotorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx LeftBack = null;
    private DcMotorEx RightBack = null;
    int Right;
    int Left;




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Code Version = Auto");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        LeftBack = hardwareMap.get(DcMotorEx.class, "LeftBack");
        RightBack = hardwareMap.get(DcMotorEx.class, "RightBack");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the batter
        //ServoLeftRoller.setDirection(CRServo.Direction.FORWARD);
        //ServoRightRoller.setDirection(CRServo.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Reset the encoder during initialization
        LeftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



        // Set the motor's target position to 300 ticks
        LeftBack.setTargetPosition(3000);

        // Switch to RUN_TO_POSITION mode
        LeftBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        LeftBack.setVelocity(200);


        // Reset the encoder during initialization
        RightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);



        // Set the motor's target position to 300 ticks
        RightBack.setTargetPosition(3000);

        // Switch to RUN_TO_POSITION mode
        RightBack.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        RightBack.setVelocity(200);

        // run until the end of the match (driver presses STOP)
            // While the Op Mode is running, show the motor's status via telemetry
            while (opModeIsActive()) {

                if(RightBack.isBusy())
                {
                    Left++;
                }
                if(LeftBack.isBusy())
                {
                    Right++;
                }
                telemetry.addData("velocityLeft", LeftBack.getVelocity());
                telemetry.addData("positionLeft", LeftBack.getCurrentPosition());
                telemetry.addData("is at target Left", !LeftBack.isBusy());
                telemetry.addData("Timer Left", Left);
                telemetry.addData("velocityRight", RightBack.getVelocity());
                telemetry.addData("positionRight", RightBack.getCurrentPosition());
                telemetry.addData("is at target Right", !RightBack.isBusy());
                telemetry.addData("Timer Left", Right);
                telemetry.update();
            }



    }
}
