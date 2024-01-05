package frc.robot.subsystems;

import java.util.LinkedList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.networktables.TimestampedInteger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.NAVX;

public class VisionSubsystem extends SubsystemBase {

    private DoubleArraySubscriber[][] vecSubscribers;
    private IntegerSubscriber[] idSubscribers;
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();

    public VisionSubsystem() {
        vecSubscribers = new DoubleArraySubscriber[4][2];
        idSubscribers = new IntegerSubscriber[4];

        for(int a = 0; a < 4; a++) {
            idSubscribers[a] = inst.getTable("fisheye").getIntegerTopic(
                Constants.Vision.TOPIC_NAMES[a][3]).subscribe(Constants.Vision.ID_DEFAULT_VALUE);
            for(int b = 0; b < 2; b++) {
                vecSubscribers[a][b] = inst.getTable("fisheye").getDoubleArrayTopic(
                        Constants.Vision.TOPIC_NAMES[a][b]).subscribe(Constants.Vision.VECTOR_DEFAULT_VALUE);
            }
        }
    }

    private Pose2d translateToFieldPose(double[] translation, double[] rotation, int tagId, int camera) {
        int count = 0;
        Matrix<N3, N3> rotMatrix = new Matrix<>(Nat.N3(), Nat.N3());
        for (int a = 0; a < 3; a++) {
            for (int b = 0; b < 3; b++) {
                rotMatrix.set(b, a, rotation[count]);
                count++;
            }
        }
        rotMatrix.times(-1);

        Matrix<N3, N1> transVec = new Matrix<>(Nat.N3(), Nat.N1());
        transVec.set(0, 0, translation[0]);
        transVec.set(1, 0, translation[1]);
        transVec.set(2, 0, translation[2]);
        Matrix<N3, N1> tagToCamTrans = rotMatrix.times(transVec);

        double hypotenuse = Math.sqrt(Math.pow(tagToCamTrans.get(2, 0), 2) + Math.pow(tagToCamTrans.get(0, 0), 2));
        double hypAngle = Constants.Vision.TAG_POSES_METERS[tagId].getRotation().getRadians() + 
            Math.atan(tagToCamTrans.get(0, 0)/tagToCamTrans.get(0, 2));

        return new Pose2d(hypotenuse * Math.cos(hypAngle), hypotenuse * Math.sin(hypAngle), new Rotation2d(NAVX.get().getYaw()));
    }

    @Override
    public void periodic() {
        for(int a = 0; a < 4; a++) {
            TimestampedDoubleArray[] tvec = vecSubscribers[a][0].readQueue();
            TimestampedDoubleArray[] rvec = vecSubscribers[a][1].readQueue();
            TimestampedInteger[] ids = idSubscribers[a].readQueue();
            for(int b = 0; b < ids.length; b++) {
                if(rvec[b].timestamp == tvec[b].timestamp && tvec[b].timestamp == ids[b].timestamp) {
                    Pose2d pose = translateToFieldPose(tvec[b].value, rvec[b].value, (int) ids[a].value, a);
                    if (pose.getX() > 0 && pose.getX() < Constants.Vision.FIELD_WIDTH_METERS && 
                        pose.getY() > 0 && pose.getY() < Constants.Vision.FIELD_LENGTH_METERS &&
                        Math.abs(pose.getX() - drivetrain.getposemethod().getX()) < Constants.Vision.MAX_MEASUREMENT_DIFFERENCE_METERS && //TODO: Replace "drivetrain.getposemethod()" with an actual pose get method
                        Math.abs(pose.getY() - drivetrain.getposemethod().getY()) < Constants.Vision.MAX_MEASUREMENT_DIFFERENCE_METERS) { //TODO: Replace "drivetrain.getposemethod()" with an actual pose get method
                            drivetrain.addmeasurementmethod(pose, (double) ids[b].timestamp); // TODO: Replace with actual method to add the measurement
                    }
                }
            }
        }
    }
}