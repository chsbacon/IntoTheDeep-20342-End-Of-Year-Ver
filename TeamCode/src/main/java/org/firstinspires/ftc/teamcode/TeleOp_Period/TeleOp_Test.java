package org.firstinspires.ftc.teamcode.TeleOp_Period;

import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.D_PAD_SPEED;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.SMOOTH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.Drive_Train_Classes.Drive_Train_Methods;
import org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Setup;

@TeleOp(name="TeleOp_Test")
//LinearOpMode is the class used to communicate to the drive hub
public class TeleOp_Test extends LinearOpMode {

    double  rotation;
    double X, Y, R;
    double lastLFPower, lastRFPower, lastLBPower, lastRBPower;
    ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    //runOpMode is called when you initialize the class on the drive hub
    public void runOpMode() throws InterruptedException {
        //Go to Drive_Methods to learn about this object and its implementation
        Drive_Train_Methods driveMethods = new Drive_Train_Methods();

        //Go to Drive_Train_Setup to learn about this object and its implementations
        Drive_Train_Setup driveTrainSetup = new Drive_Train_Setup(hardwareMap);

        //Called when you press play on the drive hub
        waitForStart();

        runTime.reset();
        driveTrainSetup.imu.resetYaw();

        //Target of the PID set to 0 in order to give the robot its starting position
        double target = 0;
        //while the robot is int and is go
        while (opModeIsActive()){

            double max;

            //Gets the robots orientation in degrees
            YawPitchRollAngles angles =  driveTrainSetup.imu.getRobotYawPitchRollAngles();
            double state = -angles.getYaw(AngleUnit.DEGREES);

            //All of the lines that relate to drive so 54 - 143 will be moved to the Drive class
            //Aids in precision by removing small offsets that the driver may accidentally input
            if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1)
                Y = -gamepad1.left_stick_y;
            else
                Y = 0;

            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1)
                X = gamepad1.left_stick_x;
            else
                X = 0;

            //Sets the power of rotation equal to the PID value returned after setting the target
            //to the degrees of rotation of the right stick of the gamepad
            //Go to the PID method for more info
            if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0)
                target = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x)) + 90;
            R = driveMethods.pidControl(target, state);

            //Uses the constant D_PAD_SPEED to allow for slow and precise movement
            if (gamepad1.dpad_up){
                Y = D_PAD_SPEED;
                X = 0;
            } else if (gamepad1.dpad_down){
                Y = -D_PAD_SPEED;
                X = 0;
            } else if (gamepad1.dpad_left){
                Y = 0;
                X = -D_PAD_SPEED;
            } else if (gamepad1.dpad_right) {
                Y = 0;
                X = D_PAD_SPEED;
            }

           //Creates the object for Field Oriented Drive go to the method for more information on FOD
            double[] fieldOriented = driveMethods.FOD(X,Y,state);

            //Change X and Y power based on the changes FOD made
            X = fieldOriented[0];
            Y = fieldOriented[1];

            //Checks for NAN as NAN is not equal to itself
            if (X!=X)
                X = 0;
            if (Y!=Y)
                Y = 0;
            if (R!=R)
                R = 0;

            //This is a simple way to use mecanum wheels assigning it wheel its own power based off X, Y, and R
            double LFPower  = Y + X + R;
            double RFPower = Y - X - R;
            double LBPower   = Y - X + R;
            double RBPower  = Y + X - R;

            //Finds the largest value out of all of the powers
            max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
            max = Math.max(max, Math.abs(LBPower));
            max = Math.max(max, Math.abs(RBPower));

            //Using the largest power maxes out the power at 1.00 and adjusts the other according to that same max power
            if (max > 1.0) {

                LFPower /= max;
                RFPower /= max;
                LBPower /= max;
                RBPower /= max;

            }

            //Smoothing code that helps make the control of the robot feel smoother
            LFPower = (1 - SMOOTH) * lastLFPower + SMOOTH * LFPower;
            RFPower = (1 - SMOOTH) * lastRFPower + SMOOTH * RFPower;
            LBPower = (1 - SMOOTH) * lastLBPower + SMOOTH * LBPower;
            RBPower = (1 - SMOOTH) * lastRBPower + SMOOTH * RBPower;
            //this sets up the drive train
            driveTrainSetup.LFMotor.setPower(LFPower);
            driveTrainSetup.RFMotor.setPower(RFPower);
            driveTrainSetup.LBMotor.setPower(LBPower);
            driveTrainSetup.RBMotor.setPower(RBPower);

            //Variables used in the calculation of the smoothing code
            lastLFPower = LFPower;
            lastRFPower = RFPower;
            lastLBPower = LBPower;
            lastRBPower = RBPower;

            //rotation is the variable used to power the two arm servos
            rotation = gamepad2.right_stick_y;
            //driveTrainSetup set up for servo arms
            driveTrainSetup.armServo1.setPower(-rotation);
            driveTrainSetup.armServo2.setPower(rotation);

            //We will move this to Drive_States and implement them as different interfaces to start creating state machines
            //Determines whether to open or close the claw servo
            if(gamepad2.left_bumper || gamepad2.left_trigger > 0.3)
                driveTrainSetup.grabber.setPosition(CLAW_OPEN);
            else if(gamepad2.right_bumper || gamepad2.right_trigger > 0.3)
                driveTrainSetup.grabber.setPosition(CLAW_CLOSED);

            //Telemetry added into the drive hub we will move this to Drive_Telemetry
            telemetry.addData("Target Grabber Angle: ", "%4.2f", driveTrainSetup.grabber.getPosition());
            telemetry.addData("Arm Power:", "%4.2f", driveTrainSetup.armServo1.getPower());
            telemetry.addData("Status", "Time Elapsed: " + runTime);

            telemetry.addData("Front Left/Right", "%4.2f, %4.2f", LFPower, RFPower);
            telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", LBPower, RBPower);

            telemetry.addData("Target PID/Current", "%4.2f, %4.2f", driveMethods.angleWrap(target), state);
            telemetry.addData("power/lastError", "%4.2f, %4.2f", R, driveMethods.getLastError());

            telemetry.update();
        }
    }
}