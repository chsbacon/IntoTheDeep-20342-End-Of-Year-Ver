package org.firstinspires.ftc.teamcode.drive.Setup_Classes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

//No need for a constructor as the variables are always constant
//To utilize import the pathway to this class and then the desired variable
//eg: import static org.firstinspires.ftc.teamcode.Setup_Classes.Drive_Train_Constants.SMOOTH;
public class Drive_Train_Constants {

    RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
    RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    public static double SMOOTH = .33;
    public static final double CLAW_OPEN = .5, CLAW_CLOSED = 0;
    public static final double Kp = .005, Ki = .00005, Kd = .8;
    public static double D_PAD_SPEED = .35;

}
