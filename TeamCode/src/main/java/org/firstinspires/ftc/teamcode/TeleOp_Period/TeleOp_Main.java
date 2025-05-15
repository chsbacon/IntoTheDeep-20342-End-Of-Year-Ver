package org.firstinspires.ftc.teamcode.TeleOp_Period;

import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.D_PAD_SPEED;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.Kd;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.Ki;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.Kp;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.SMOOTH;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Setup;

//NEVER TOUCH THE MAIN CLASS PLZ GO TO TEST
@TeleOp(name="TeleOp_Main")
public class TeleOp_Main extends LinearOpMode {
    double integralSum = 0;
    double lastError, rotation;
    ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double X, Y, R;
    double lastLFPower, lastRFPower, lastLBPower, lastRBPower;
    ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

        Drive_Train_Setup driveTrainSetup = new Drive_Train_Setup(hardwareMap);

        waitForStart();

        runTime.reset();
        driveTrainSetup.imu.resetYaw();
        double target = 0;

        while (opModeIsActive()){

            double max;

            YawPitchRollAngles angles =  driveTrainSetup.imu.getRobotYawPitchRollAngles();
            double state = -angles.getYaw(AngleUnit.DEGREES);

            if (gamepad1.left_stick_y > .15 || gamepad1.left_stick_y < -.15) {
                Y = -gamepad1.left_stick_y;
            } else {
                Y = 0;
            }

            if (gamepad1.left_stick_x > .15 || gamepad1.left_stick_x < -.15){
                X = gamepad1.left_stick_x;
            } else {
                X = 0;
            }

            if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                target = Math.toDegrees(Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x)) + 90;
            }
            R = pidControl(target, state);

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

            double[] pp=FOD(X,Y,state);

            X=pp[0];
            Y=pp[1];

            if (X!=X)
                X = 0;
            if (Y!=Y)
                Y = 0;
            if (R!=R)
                R = 0;

            double LFPower  = Y + X + R;
            double RFPower = Y - X - R;
            double LBPower   = Y - X + R;
            double RBPower  = Y + X - R;

            max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
            max = Math.max(max, Math.abs(LBPower));
            max = Math.max(max, Math.abs(RBPower));

            if (max > 1.0) {

                LFPower /= max;
                RFPower /= max;
                LBPower /= max;
                RBPower /= max;

            }

            LFPower = (1 - SMOOTH) * lastLFPower + SMOOTH * LFPower;
            RFPower = (1 - SMOOTH) * lastRFPower + SMOOTH * RFPower;
            LBPower = (1 - SMOOTH) * lastLBPower + SMOOTH * LBPower;
            RBPower = (1 - SMOOTH) * lastRBPower + SMOOTH * RBPower;

            driveTrainSetup.LFMotor.setPower(LFPower);
            driveTrainSetup.RFMotor.setPower(RFPower);
            driveTrainSetup.LBMotor.setPower(LBPower);
            driveTrainSetup.RBMotor.setPower(RBPower);

            lastLFPower = LFPower;
            lastRFPower = RFPower;
            lastLBPower = LBPower;
            lastRBPower = RBPower;

            rotation = gamepad2.right_stick_y;

            driveTrainSetup.armServo1.setPower(-rotation);
            driveTrainSetup.armServo2.setPower(rotation);

            if(gamepad2.left_bumper || gamepad2.left_trigger > 0.3)
                driveTrainSetup.grabber.setPosition(CLAW_OPEN);
            else if(gamepad2.right_bumper || gamepad2.right_trigger > 0.3)
                driveTrainSetup.grabber.setPosition(CLAW_CLOSED);

            telemetry.addData("Target Grabber Angle: ", "%4.2f", driveTrainSetup.grabber.getPosition());
            telemetry.addData("Arm Power:", "%4.2f", driveTrainSetup.armServo1.getPower());
            telemetry.addData("Status", "Time Elapsed: " + runTime);

            telemetry.addData("Front Left/Right", "%4.2f, %4.2f", LFPower, RFPower);
            telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", LBPower, RBPower);

            telemetry.addData("Target PID/Current", "%4.2f, %4.2f", angleWrap(target), state);
            telemetry.addData("power/lastError", "%4.2f, %4.2f", R, lastError);

            telemetry.update();
        }
    }

    public double pidControl(double target, double current){

        double error = angleWrap((target - current));
        integralSum += error * pidTimer.time();
        double derivative = (error - lastError) / pidTimer.milliseconds();
        lastError = error;

        if(integralSum > 2000){
            integralSum = 2000;
        }
        if(integralSum < -2000){
            integralSum = -2000;
        }

        pidTimer.reset();

        return (((error * Kp) + (derivative * Kd) + (integralSum * Ki)));
    }

    public static double angleWrap (double degrees){
        while (degrees > 180){
            degrees -= 360;
        }
        while (degrees < -180){
            degrees += 360;
        }
        return degrees;
    }

    public static double[] FOD(double x, double y, double state) {
        double r = Math.sqrt(x * x + y * y);
        if(r!=r)
            r=0;

        double theta = Math.toDegrees(Math.atan2(y, x)) + state;
        if (theta > 180) theta -= 360;
        else if (theta < -180) theta += 360;
        double thetaRad = Math.toRadians(theta);
        return new double[]{Math.abs((r * Math.cos(thetaRad)))<0.01 ? 0.0:(r * Math.cos(thetaRad)), Math.abs((r * Math.sin(thetaRad)))<0.01 ? 0.0:(r * Math.sin(thetaRad))};
    }
}