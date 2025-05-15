package org.firstinspires.ftc.teamcode.drive.Drive_Train_Classes;

import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.Kd;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.Ki;
import static org.firstinspires.ftc.teamcode.drive.Setup_Classes.Drive_Train_Constants.Kp;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive_Train_Methods {
    private double integralSum = 0, lastError;
    ElapsedTime pidTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    //All methods relating to the driving of the robot
    //Takes the target angle and using calculations based on the current angle return the power needed for rotation
    //Requires tuning based on any new major changes to the robot
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

    //Helps keep the angle on a 180 to -180 range
    public double angleWrap (double degrees){
        while (degrees > 180){
            degrees -= 360;
        }
        while (degrees < -180){
            degrees += 360;
        }
        return degrees;
    }

    //FOD (Field Oriented Drive_Train) adds the rotation of the left joystick to the rotation of the robot
    //This allows us to always move the robot based off of only the human driver
    //I coded this part so I know it works, its chill
    public double[] FOD(double x, double y, double state) {
        double r = Math.sqrt(x * x + y * y);
        if(r!=r)
            r=0;

        double theta = Math.toDegrees(Math.atan2(y, x)) + state;
        if (theta > 180) theta -= 360;
        else if (theta < -180) theta += 360;
        double thetaRad = Math.toRadians(theta);
        return new double[]{Math.abs((r * Math.cos(thetaRad)))<0.01 ? 0.0:(r * Math.cos(thetaRad)), Math.abs((r * Math.sin(thetaRad)))<0.01 ? 0.0:(r * Math.sin(thetaRad))};
    }

    // Getter methods for the integralSum and lastError variables
    public double getIntegralSum() {
        return integralSum;
    }

    public double getLastError() {
        return lastError;
    }
}

