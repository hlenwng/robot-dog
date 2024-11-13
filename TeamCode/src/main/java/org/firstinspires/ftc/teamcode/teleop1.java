package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="teleop1", group="Robot")
//@Disabled
public class teleop1 extends OpMode {

    public final double L1 = 90.0; // Hip length in mm
    public final double L2 = 111.93; // Shin length in mm

    public class ServoPositions {
        public int leg;
        public int direct;
        public int gear;

        public ServoPositions(int leg, int direct, int gear) {
            this.leg = leg;
            this.direct = direct;
            this.gear = gear;
        }
    }

    public final ServoPositions LF = new ServoPositions(0,1,2);
    public final ServoPositions RF = new ServoPositions(3,4,5);
    public final ServoPositions LB = new ServoPositions(6,7,8);
    public final ServoPositions RB = new ServoPositions(9,10,11);

    public class DogLeg {
        private Servo[] servos;

        public DogLeg(HardwareMap hardwareMap) {
            servos = new Servo[12];
            servos[0] = hardwareMap.get(Servo.class, "LF_leg");
            servos[1] = hardwareMap.get(Servo.class, "LF_direct");
            servos[2] = hardwareMap.get(Servo.class, "LF_gear");

            servos[3] = hardwareMap.get(Servo.class, "RF_leg");
            servos[4] = hardwareMap.get(Servo.class, "RF_direct");
            servos[5] = hardwareMap.get(Servo.class, "RF_gear");

            servos[6] = hardwareMap.get(Servo.class, "LB_leg");
            servos[7] = hardwareMap.get(Servo.class, "LB_direct");
            servos[8] = hardwareMap.get(Servo.class, "LB_gear");

            servos[9] = hardwareMap.get(Servo.class, "RB_leg");
            servos[10] = hardwareMap.get(Servo.class, "RB_direct");
            servos[11] = hardwareMap.get(Servo.class, "RB_gear");
        }

        // Convert angle to servo position (0-1 range for FTC Servos)
        private double angleToTick(double angleInRad) {
            //Servo ticks from 0 to 255
            //Dog angles from -150 to 150 degrees

            double angleInDeg = angleInRad * (180.0/PI);

            double minAngle = -150.0;
            double maxAngle = 150.0;
            double servoPos = Range.scale(angleInDeg, minAngle, maxAngle, 0.0, 255.0);
            return Range.clip(servoPos, 0.0, 1.0);
        }

        // Helper to move leg to position based on angles
        public void setPos(ServoPositions servos, double q1_deg, double q2_deg) {
            servos.leg.setPosition(angleToTick(q1_deg));
            servos.gear.setPosition(angleToTick(q2_deg));
        }

        // IK function for calculating angles
        private double[] calculateAngles(double x, double y) {
            double q1, q2;
            double sq_euc_dist = (x * x) + (y * y);
            double c2 = (sq_euc_dist - (L1 * L1) - (L2 * L2)) / (2 * L1 * L2);

            if (Math.abs(c2) > 1) {
                return new double[]{Double.NaN, Double.NaN};
            }

            q2 = Math.acos(c2);
            double theta = Math.atan2(y, x);
            q1 = theta - Math.atan2(L2 * Math.sin(q2), L1 + L2 * Math.cos(q2));

            return new double[]{Math.toDegrees(q1), Math.toDegrees(q2)};
        }

        public void setGait(String gaitName) {
            //walking, trotting, cantering and galloping

            if (gaitName == "INIT") {

            }
            else if (gaitName == "WALK_FORWARD") {
                legs.setPos(0, 20);
            }
            else if (gaitName == "WALK_BACK") {

            }

            else if (gaitName == "CANTER") {

            }
        }
    }

    private DogLeg legs;

    @Override
    public void init() {
        legs = new DogLeg(hardwareMap);

        legs.setGait("INIT");
    }

    @Override
    public void loop() {
        // Control the leg based on gamepad input
        double left_y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!

        if (left_y < -0.1){
            legs.setGait("WALK_FORWARD");
        }

        if (left_y > 0.1) {
            legs.setGait("WALK_BACK");
        }

    }



}