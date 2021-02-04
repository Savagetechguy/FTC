package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AutoMovement extends LinearOpMode{

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
        LeftShooter.setDirection(DcMotor.Direction.REVERSE); //Can be changed based on motor configuration
        RightShooter.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)


        // run until the end of the match (driver presses STOP)


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
            double LeftForward = -gamepad1.left_stick_y;
            double LeftSide  =  gamepad1.left_stick_x;
            double RightSide  =  gamepad1.right_stick_x;
            boolean Shoot = gamepad2.a;
            boolean Collect = gamepad2.b;
            boolean ElevatorControl = gamepad2.x;
            int CollectasInt;
            int ShootasInt;
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
             public AutoMovement(int milliseconds) {
                leftBackDrive.setPower(1);
                rightBackDrive.setPower(1);
                leftFrontDrive.setPower(1;
                rightFrontDrive.setPower(1);
                sleep(milliseconds);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
            }


        }
