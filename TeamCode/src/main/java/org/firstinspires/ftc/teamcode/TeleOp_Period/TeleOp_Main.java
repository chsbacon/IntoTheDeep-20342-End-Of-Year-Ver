package org.firstinspires.ftc.teamcode.TeleOp_Period;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
//adding test for pushing to github
@TeleOp(name="TeleOp_Main")
public class TeleOp_Main extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    IMU imu;
    DcMotorEx LFmotor;
    DcMotorEx RFmotor;
    DcMotorEx LBmotor;
    DcMotorEx RBmotor;
    RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    @Override
    public void runOpMode() throws InterruptedException {

        LFmotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        RFmotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        LBmotor = hardwareMap.get(DcMotorEx.class, "LBmotor");
        RBmotor = hardwareMap.get(DcMotorEx.class, "RBmotor");

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
        double state=0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){
            double max;
            YawPitchRollAngles angles =  imu.getRobotYawPitchRollAngles();
            state = -angles.getYaw(AngleUnit.DEGREES);
            double Y = -gamepad1.left_stick_y;
            double X = gamepad1.left_stick_x;
            double R = gamepad1.right_stick_x;
            double[]dd=changeMe(X,Y,state);
            X=dd[0];
            Y=dd[1];
            double LFPower  = Y + X + R;
            double RFPower = Y - X - R;
            double LBPower   = Y - X + R;
            double RBPower  = Y + X - R;

            max = Math.max(Math.abs(LFPower), Math.abs(RFPower));
            max = Math.max(max, Math.abs(LBPower));
            max = Math.max(max, Math.abs(RBPower));

            if (max > 1.0) {

                LFPower  /= max;
                RFPower /= max;
                LBPower   /= max;
                RBPower  /= max;

            }

            double LFHun = LFPower * 100;
            double LBHun = LBPower * 100;
            double RFHun = RFPower * 100;
            double RBHun = RBPower * 100;

            if (LFHun >= 1){
                LFPower = (Math.log(LFHun) / Math.log(100)) * 1;
            } else if (LFHun <= -1){
                LFPower = (Math.log(LFHun*-1) / Math.log(100)) * -1;
            }

            if (RFHun >= 1){
                RFPower = (Math.log(RFHun) / Math.log(100)) * 1;
            } else if (RFHun <= -1){
                RFPower = (Math.log(RFHun*-1) / Math.log(100)) * -1;
            }

            if (LBHun >= 1){
                LBPower = (Math.log(LBHun) / Math.log(100)) * 1;
            } else if (LBHun <= -1){
                LBPower = (Math.log(LBHun*-1) / Math.log(100)) * -1;
            }

            if (RBHun >= 1){
                RBPower = (Math.log(RBHun) / Math.log(100)) * 1;
            } else if (RBHun <= -1){
                RBPower = (Math.log(RBHun*-1) / Math.log(100)) * -1;
            }

            LFmotor.setPower(LFPower);
            RFmotor.setPower(RFPower);
            LBmotor.setPower(LBPower);
            RBmotor.setPower(RBPower);

            telemetry.addData("Status", "Time Elapsed: " + runtime.toString());
            telemetry.addData("Front Left/Right", "%4.2f, %4.2f", LFPower, RFPower);
            telemetry.addData("Back  Left/Right", "%4.2f, %4.2f", LBPower, RBPower);
            telemetry.addData("State",state);
            telemetry.update();
        }
    }
    public static double[] changeMe(double x, double y,double state){
        double r=Math.sqrt(x*x+y*y);
        double A=state;
        double A1=180-A;
        double a=x*Math.toDegrees(Math.cos(A))-y*Math.toDegrees(Math.cos(A));
        double b=x*Math.toDegrees(Math.cos(A))+y*Math.toDegrees(Math.cos(A));
        double a1=x*Math.toDegrees(Math.cos(A1))-y*Math.toDegrees(Math.cos(A1));
        double b1=x*Math.toDegrees(Math.cos(A1))+y*Math.toDegrees(Math.cos(A1));
        if(y!=0)
            x=(a+a1)/(y*(a+a1));
        else
            x=(a+a1);
        if(x!=0)
            y=(b+b1)/(x*(b+b1));
        else
            y=b+b1;
        double[]gg={x,y};
        return gg;

    }
}
