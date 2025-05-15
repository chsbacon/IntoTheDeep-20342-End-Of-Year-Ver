package org.firstinspires.ftc.teamcode.drive.Setup_Classes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
//This class is used to help encapsulate our variable and objects used within our Teleop and Auto periods
//To implement this code simply declare a new variable as an object
//eg: Drive_Train_Setup driveTrainSetup = new Drive_Train_Setup(hardwareMap);
//This must be declared within the runOpMode method of a LinearOpMode child class
//Then simply use the name of your new object variable to utilize the motors and other declared devices
//eg: driveTrainSetup.LFMotor.setPower(LFPower);

//The class extends the constants class in order to quickly use variables declared such as LOGO_FACING_DIR
public class Drive_Train_Setup extends Drive_Train_Constants {

    public Servo grabber;
    public CRServo armServo1, armServo2;
    public DcMotorEx LFMotor, RFMotor, LBMotor, RBMotor;
    public IMU imu;

    //This is how you declare a constructor
    //Passing the HardwareMap in as a parameter allows us to
    //grab the motors from the driver hub and initialize them
    public Drive_Train_Setup(HardwareMap hardwareMap){

        grabber = hardwareMap.get(Servo.class, "grabber");
        armServo1 = hardwareMap.get(CRServo.class, "armServo1");
        armServo2 = hardwareMap.get(CRServo.class, "armServo2");

        LFMotor = hardwareMap.get(DcMotorEx.class, "LFmotor");
        LFMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor = hardwareMap.get(DcMotorEx.class, "RFmotor");
        RFMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor = hardwareMap.get(DcMotorEx.class, "LBmotor");
        LBMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor = hardwareMap.get(DcMotorEx.class, "RBmotor");
        RBMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        LFMotor.setDirection(DcMotorEx.Direction.REVERSE);
        LBMotor.setDirection(DcMotorEx.Direction.REVERSE);
        RFMotor.setDirection(DcMotorEx.Direction.FORWARD);
        RBMotor.setDirection(DcMotorEx.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot
                (LOGO_FACING_DIR, USB_FACING_DIR));
        imu.initialize(parameters);

    }
}
