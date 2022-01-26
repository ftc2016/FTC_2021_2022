package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test")
public class LeftStick2 extends OpMode {
    public DcMotor leftFront, leftRear, rightRear, rightFront;
    public DcMotor armRotation, armRevolution, carousel;
    public CRServo intake;
    public Servo index;
    boolean Armpos = true;


    // above initializes all the aspects we need to make our robot function
    @Override
    public void init() {
        // defining all the hardware
        //Wheels
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        //Carousel
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        //Arm
        armRotation = hardwareMap.get(DcMotor.class, "armRot");
        armRevolution = hardwareMap.get(DcMotor.class, "armRev");
        intake = hardwareMap.get(CRServo.class, "intake");
        index = hardwareMap.get(Servo.class, "index");
        //Reversing inverted motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Teleop Status", "Ready to Deploy");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Teleop Status", "Running Teleop");
        telemetry.update();
        if(Armpos == true){
            telemetry.addData("Arm Mode:", "Control");
            telemetry.update();
        }else if(Armpos == false){
            telemetry.addData("Arm Mode:", "Position");
            telemetry.update();
        }else{
            telemetry.addData("Arm Mode:", "Unknown mode/Holding Arm");
            telemetry.update();
        }
        telemetry.addData("Arm Encoder Value:", armRotation.getCurrentPosition());
        telemetry.update();

        if (gamepad2.dpad_left) {
            Armpos = !Armpos;
        }
        double leftFrontPower = (0.8 * gamepad1.left_stick_y);
        double leftRearPower = (0.8 * gamepad1.right_stick_x);
        double rightFrontPower = (0.8 * gamepad1.right_stick_x);
        double rightRearPower = (0.8 * gamepad1.left_stick_y);

        if (gamepad1.left_trigger > 0.1) {
            leftFrontPower = -0.75 * gamepad1.left_trigger;
            rightRearPower = 0.75 * gamepad1.left_trigger;
        }

        if (gamepad1.right_trigger > 0.1) {
            leftFrontPower = 0.75 * gamepad1.right_trigger;
            rightRearPower = -0.75 * gamepad1.right_trigger;
        }

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        //Hold X motor position when no power supplied
        if (gamepad1.left_stick_y <= 0.05 && gamepad1.left_trigger <= 0.05 && gamepad1.right_trigger <= 0.05)
        {
            leftFront.setPower(0);
            rightRear.setPower(0);
        }

        //Hold Y motor position when no power supplied
        if (gamepad1.right_stick_x == 0 && gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0)
        {
            rightFront.setPower(0);
            leftRear.setPower(0);
        }


        double armRotpwr = -gamepad2.right_stick_y/1.5;
        armRevolution.setPower(gamepad2.right_stick_x/1.25);

        if(gamepad2.y){
            armRotation.setPower(0);
        }
        if(gamepad2.dpad_right){
            armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(Armpos == true){
            armRotation.setPower(armRotpwr);
            if(gamepad2.dpad_right){
                armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        } else if(Armpos == false){
            if (gamepad2.a){
                telemetry.addData("Arm Pos Status:", "Running to Ground");
                telemetry.update();
                armRotation.setTargetPosition(0);

                armRotation.setPower(0.8);

                armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (armRotation.isBusy()) {
                    if (gamepad2.dpad_left) {
                        Armpos = !Armpos;
                    }
                    if (Armpos == true || gamepad2.y) {
                        armRotation.setPower(0);
                        break;
                    } else {}
                }
                armRotation.setPower(0);

            }else if (gamepad2.b) {
                telemetry.addData("Arm Pos Status:", "Running to Top Level");
                telemetry.update();
                armRotation.setTargetPosition(0);

                armRotation.setPower(0.8);

                armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (armRotation.isBusy()){
                    if (gamepad2.dpad_left) {
                        Armpos = !Armpos;
                    }
                    if (Armpos == true || gamepad2.y) {
                        armRotation.setPower(0);
                        break;
                    } else {}
                }
                armRotation.setPower(0);
            }else{
                armRotation.setPower(0);
            }
        }else{
            armRotation.setPower(0);
        }


        if (gamepad1.dpad_up == true){
            carousel.setPower(0.8);
        } else if (gamepad2.dpad_up == true){
            carousel.setPower(0.8);
        } else{
            if (gamepad1.dpad_down == true){
                carousel.setPower(-0.8);
            } else if (gamepad2.dpad_down == true){
                carousel.setPower(-0.8);
            } else{
                carousel.setPower(0);
            }
        }

        if (gamepad2.x) {
            index.setPosition(0.4);
        }else{
            index.setPosition(0.9);
        }

        if(gamepad2.right_trigger >= 0.05 || gamepad2.left_trigger >= 0.05){
            if (gamepad2.right_trigger >= 0.05 && gamepad2.left_trigger >= 0.05) {
                index.setPosition(0.4);
            }

            if (gamepad2.right_trigger >= 0.05 && gamepad2.left_trigger <= 0.05) {
                intake.setPower(1);
            } else if (gamepad2.left_trigger >= 0.05 && gamepad2.right_trigger <= 0.05) {
                intake.setPower(-1);
            }
        }else{
            intake.setPower(0);
        }

    }
    @Override
    public void stop()
    {
    }
}

