package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "frontLeftMotor";
        FollowerConstants.leftRearMotorName = "backLeftMotor";
        FollowerConstants.rightFrontMotorName = "frontRightMotor";
        FollowerConstants.rightRearMotorName = "backRightMotor";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 12.7;

        FollowerConstants.xMovement = 54.819816518326164;
        FollowerConstants.yMovement = 40.30794971388268;
        FollowerConstants.forwardZeroPowerAcceleration = -75.98040471152052;
        FollowerConstants.lateralZeroPowerAcceleration = -100.6162336826837;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(.15,0,0,0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0,0,0);
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.0005,0.6,0);
        FollowerConstants.centripetalScaling = 0.0005;
    }
}
