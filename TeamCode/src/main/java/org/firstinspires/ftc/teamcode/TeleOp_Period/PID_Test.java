package org.firstinspires.ftc.teamcode.TeleOp_Period;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name="TeleOp_Main_PID")
public class PID_Test extends LinearOpMode {

    Servo grabber;
    CRServo armServo1, armServo2;
    boolean stop = false;
    double rotation;
    double targetAngle = 0.0;
    final double MAX_BOUND = 1, MIN_BOUND = 0, OPEN = 0.5, CLOSED = 0;
    double integralSum = 0;
    final double p = .005;
    final double i = .00005;
    final double d = .8;

    double lastError;
    IMU imu;
    ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    DcMotorEx LFmotor;
    DcMotorEx RFmotor;
    DcMotorEx LBmotor;
    DcMotorEx RBmotor;

    RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    final double D_PAD_SPEED = .3;

    double X,Y;
    double R;
    ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void runOpMode() throws InterruptedException {

        grabber = hardwareMap.get(Servo.class, "grabber");
        armServo1 = hardwareMap.get(CRServo.class, "armServo1");
        armServo2 = hardwareMap.get(CRServo.class, "armServo2");

        LFmotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        LFmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RFmotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        RFmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LBmotor = hardwareMap.get(DcMotorEx.class, "LBmotor");
        LBmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RBmotor = hardwareMap.get(DcMotorEx.class, "RBmotor");
        RBmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        LFmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RFmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot
                (LOGO_FACING_DIR, USB_FACING_DIR));
        imu.initialize(parameters);

        telemetry.addData("Status:", " Ready");
        telemetry.update();

        waitForStart();

        runTime.reset();
        imu.resetYaw();

        double target = 0;

        while (opModeIsActive()){
            double max;
            target = angleWrap(target);
            YawPitchRollAngles angles =  imu.getRobotYawPitchRollAngles();
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
            if (gamepad1.right_stick_x == 0 && gamepad1.right_stick_y==0) {
                R = pidControl(target, state);
            } else{
                target = Math.toDegrees(Math.atan2(gamepad1.right_stick_y,gamepad1.right_stick_x)) + 90;
                R = pidControl(target, state);
            }

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
            if(gamepad1.a){

                boolean isNegative = false;

                int card=90;
                double min=Math.abs(state-90);

                if(angleWrap(state) < -45 || angleWrap(state) > -135)
                    isNegative = true;

                if(min>Math.abs(state-180)){
                    card = 180;
                    min=Math.abs(state-180);
                }

                if(min>Math.abs(state)) {
                    card=0;
                }

                if (isNegative)
                    card *= -1;
                target=card;
            }

            double[] pp=FOD(X,Y,state);
            X=pp[0];
            Y=pp[1];

            if(X!=X)
                X=0;
            if(Y!=Y)
                Y=0;
            if(R!=R)
                R=0;

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

            if (Y != 0 || X != 0 && gamepad1.dpad_left && gamepad1.dpad_down && gamepad1.dpad_up && gamepad1.dpad_right) {

                double LFHun = LFPower * 100;
                double LBHun = LBPower * 100;
                double RFHun = RFPower * 100;
                double RBHun = RBPower * 100;

                LFPower = (Math.pow(LFHun, 3) / Math.pow(100, 3));
                RFPower = (Math.pow(RFHun, 3) / Math.pow(100, 3));
                LBPower = (Math.pow(LBHun, 3) / Math.pow(100, 3));
                RBPower = (Math.pow(RBHun, 3) / Math.pow(100, 3));

            }
            LFmotor.setPower(LFPower);
            RFmotor.setPower(RFPower);
            LBmotor.setPower(LBPower);
            RBmotor.setPower(RBPower);

            if (!stop) {
                if (gamepad1.x) {
                    target -= 90;
                    stop = true;
                }

                if (gamepad1.b) {
                    target += 90;
                    stop = true;
                }
            }

            if (!gamepad1.x && !gamepad1.b){
                stop = false;
            }

            rotation = gamepad2.right_stick_y;

            armServo1.setPower(-rotation);
            armServo2.setPower(rotation);

            if(gamepad2.left_bumper || gamepad2.left_trigger > 0.3)
                grabber.setPosition(OPEN);
            else if(gamepad2.right_bumper || gamepad2.right_trigger > 0.3)
                grabber.setPosition(CLOSED);

            telemetry.addData("Target Grabber Angle: ", "%4.2f", grabber.getPosition());
            telemetry.addData("Arm Power:", "%4.2f", armServo1.getPower());
            telemetry.addData("Status", "Time Elapsed: " + runTime);

            telemetry.addData("Front Left/Right", "%4.2f, %4.2f", LFPower, RFPower);
            telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", LBPower, RBPower);

            telemetry.addData("Target PID/Current", "%4.2f, %4.2f", angleWrap(target), angleWrap(state));
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

        return (((error * p) + (derivative * d) + (integralSum * i)));
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
        if (Double.isNaN(x) || Double.isNaN(y)) return new double[]{0.0, 0.0}; // Ensure no NaN input
        double r = Math.sqrt(x * x + y * y);
        double theta = Math.toDegrees(Math.atan2(y, x)) + state;

        theta = angleWrap(theta); // Keep angle in range
        double thetaRad = Math.toRadians(theta);

        double newX = r * Math.cos(thetaRad);
        double newY = r * Math.sin(thetaRad);

        return new double[]{
                Math.abs(newX) < 0.01 ? 0.0 : newX,
                Math.abs(newY) < 0.01 ? 0.0 : newY
        };
    }

}