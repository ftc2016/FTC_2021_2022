package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="REALDRIVER", group = "Default")
public class Driver extends OpMode {
    //Arm1
    private final double liftPosScale = 50;
    private final double liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private final double integrater = 0.001;
    private final double intpower = 0.00075;
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;
    //Slide
    private final double liftPosScale2 = 50;
    private final double liftPowScale2 = 0.0025;
    private final double liftPosCurrent2=0;
    private final double liftPosDes2=0;
    private final double liftPosError2=0;
    private final double liftPow2=0;
    private final double integrater2 = 0.001;
    private final double intpower2 = 0.00075;
    double multiplier2 = 1, speedK2 = 1;
    boolean turtle2 = false, sloth2 = false;
    double rotPos2 = 0, foundPos2 = 1;
    int shootPos2 = 0;

    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;


    public void init() {

        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("Pick");
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        //Forward, Back, Left, Right
        leftvertical.setPower(gamepad1.right_stick_x*-1);
        rightvertical.setPower(gamepad1.right_stick_x*-1);
        //Forward and Backward - left stick - up/down
        lefthorizontal.setPower(gamepad1.right_stick_y);
        righthorzontal.setPower(gamepad1.right_stick_y);

        //Spin
        lefthorizontal.setPower(-gamepad1.left_trigger / 2);
        righthorzontal.setPower(gamepad1.left_trigger / 2);
        leftvertical.setPower(-gamepad1.left_trigger / 2);
        rightvertical.setPower(gamepad1.left_trigger / 2);
        // Spin other way
        lefthorizontal.setPower(gamepad1.right_trigger / 2);
        righthorzontal.setPower(-gamepad1.right_trigger / 2);
        leftvertical.setPower(gamepad1.right_trigger / 2);
        rightvertical.setPower(-gamepad1.right_trigger / 2);
        //Spin Carousel
        if (gamepad1.a) {
            Top.setPower(0.2);
        } else if (gamepad1.y){
            Top.setPower(-0.2);
        }else{
            Top.setPower(0);
        }

        //Intake
        if (gamepad1.x) {
            Pick.setPower(1);
        } else if (gamepad1.b) {
            Pick.setPower(-1);
        } else {
            Pick.setPower(0);
        }

        //PID for Arm1
        liftPosCurrent = Arm1.getCurrentPosition();
        liftPosDes += speedK*liftPosScale*gamepad2.left_stick_y/2;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//      integrater += liftPosError;                                             //unnecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
        Arm1.setPower(liftPow);

        // PID for Slide
        /*liftPosCurrent2 = Slide.getCurrentPosition();
        liftPosDes2 += speedK2*liftPosScale2*gamepad2.right_stick_x;                //input scale factor
        liftPosError2 = liftPosDes2 - liftPosCurrent2;
//      integrater2 += liftPosError2;                                              //unnecessary
        liftPow2 = Math.min(Math.max(liftPowScale2*liftPosError2, -1.00), 1.00);   //proportional gain
        if(liftPow2 >= 1){ liftPosDes2 = liftPosCurrent2+(1/liftPowScale2); }       //AntiWindup Code
        if(liftPow2 <= -1) {liftPosDes2 = liftPosCurrent2-(1/liftPowScale2); }      //AntiWindup Code
        Slide.setPower(liftPow2);*/
    }
}