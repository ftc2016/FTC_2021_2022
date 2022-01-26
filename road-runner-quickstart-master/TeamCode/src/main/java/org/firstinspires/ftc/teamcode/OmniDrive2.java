package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Omni with Arm Pos")
public class OmniDrive2 extends OpMode {
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;
    //Declare Motor names
    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    DcMotor carousel;
    DcMotor carousel2;
    DcMotor Arm1;
    CRServo intake;
    Servo drop;
    boolean Armpos = false;
    boolean Armpos2 = true;

    //Arm1 PID Varibles
    private final double liftPosScale = 50;
    private final double liftPowScale = 0.0025;
    private double liftPosCurrent = 0, liftPosDes = 0, liftPosError = 0, liftPow = 0;
    private final double integrater = 0.001;
    private final double intpower = 0.00075;

    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {
        // defining all the hardware
        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("arm");
        carousel = hardwareMap.dcMotor.get("carousel");
        carousel2 = hardwareMap.dcMotor.get("carousel2");
        drop = hardwareMap.servo.get("drop");
        intake = hardwareMap.crservo.get("intake");
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        //set arm for game pad postion mode
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_left) {
            Armpos = !Armpos;
        }
        if (Armpos) {
            Armpos2 = true;
        } else {
            Armpos2 = false;
        }
        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;

        float y1 = -gamepad1.right_stick_x;
        float x1 = gamepad1.right_stick_y;
        float r1 = gamepad1.left_trigger;
        float r2 = gamepad1.right_trigger;

        boolean cw = gamepad2.right_bumper;
        boolean ccw = gamepad2.left_bumper;
        // Handle regular movement
        leftFrontPower += y1;
        leftBackPower += y1;

        // Handle strafing movement
        rightFrontPower += x1;
        rightBackPower += x1;
        //Spin
        lefthorizontal.setPower(-gamepad1.left_trigger);
        righthorzontal.setPower(gamepad1.left_trigger);
        leftvertical.setPower(-gamepad1.left_trigger);
        rightvertical.setPower(gamepad1.left_trigger);
        // Spin other way
        lefthorizontal.setPower(gamepad1.right_trigger);
        righthorzontal.setPower(-gamepad1.right_trigger);
        leftvertical.setPower(gamepad1.right_trigger);
        rightvertical.setPower(-gamepad1.right_trigger);

        //GAME PAD 2 BELOW

//Reset down pos
        if (gamepad2.dpad_down) {
            Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

//Carousel wheel controls
        if (cw == true) {
            carousel.setPower(1);
            carousel2.setPower(1);
        } else if (ccw == true) {
            carousel.setPower(-1);
            carousel2.setPower(-1);
        } else {
            carousel.setPower(0);
            carousel2.setPower(0);
        }

        Arm1.setPower(gamepad2.left_stick_y);

        //Arm Pos button
        if (Armpos2 = true) {
            if (gamepad2.b) {

                //set target position also where the arm goes
                Arm1.setTargetPosition(0);


                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                //set power and how fast the arm goes
                Arm1.setPower(0.5);

                //waiting while arm is done
                while (Arm1.isBusy()){

                }
                Arm1.setPower(0);
            }
            if (gamepad2.y) {
                //set target position
                Arm1.setTargetPosition(-1000);

                Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //set power
                Arm1.setPower(0.5);
                while (Arm1.isBusy()){
                }
                Arm1.setPower(0);
            }
        }

        //Intake and Drop Control
        if (gamepad2.a) {
            drop.setPosition(1);
        } else {
            drop.setPosition(0.5);
        }

        if (gamepad2.x) {
            intake.setPower(1);
        } else if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        // Scale movement !!!! Don't Touch
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));
        if (max > 1) {
            leftFrontPower = (float) Range.scale(leftFrontPower, -max, max, -.30, .30);
            leftBackPower = (float) Range.scale(leftBackPower, -max, max, -.30, .30);
            rightFrontPower = (float) Range.scale(rightFrontPower, -max, max, -.30, .30);
            rightBackPower = (float) Range.scale(rightBackPower, -max, max, -.30, .30);
        }
        leftvertical.setPower(-leftBackPower);
        rightvertical.setPower(-leftFrontPower);
        lefthorizontal.setPower(-rightFrontPower);
        righthorzontal.setPower(-rightBackPower);

        //PID for Arm1
        /*
        liftPosCurrent = Arm1.getCurrentPosition();
        liftPosDes += speedK * liftPosScale * gamepad2.left_stick_y;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//      integrater += liftPosError;                                             //unnecessary
        liftPow = Math.min(Math.max(liftPowScale * liftPosError, -1.00), 1.00);   //proportional gain
        if (liftPow >= 1) {
            liftPosDes = liftPosCurrent + (1 / liftPowScale);
        }       //AntiWindup Code
        if (liftPow <= -1) {
            liftPosDes = liftPosCurrent - (1 / liftPowScale);
        }      //AntiWindup Code
        Arm1.setPower(liftPow);
           }*/
    }
}