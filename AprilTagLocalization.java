package org.firstinspires.ftc.teamcode.Control.Localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalization {
    //current robot position.
    double x;
    double y;
    double h;

    //account for the cameras position, subtract as necessary.
    double cameraX = 1;
    double cameraY = 0;
    double cameraH = 0;

    ArrayList<Double> allX;
    ArrayList<Double> allY;
    ArrayList<Double> allH;
    Pose2d currentPose;
    private List<AprilTagDetection> locFrom;

    //figure out which four will be used/which ones are printed. They must be in the 36h11 family(0-20)
    private int[] fieldTags = {0,1,2,3};

    public AprilTagLocalization(List<AprilTagDetection> currentFinds) {
        locFrom = currentFinds;
    }

    public void localize() {
        for (AprilTagDetection detection : locFrom) {
            for(int tagNum : fieldTags)
                if(detection.id == tagNum) {
                    Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                    //position like odometry
                    //removed the z axis to streamline performance. Also with the assumption its unnecessary.
                    allX.add(detection.rawPose.x);
                    allY.add(detection.rawPose.y);
                    //rotation? pitch, roll, yaw
                    //y is assumed as the pitch(heading)
                    allH.add((double) rot.secondAngle);
                }
        }
        calculateAverage();
    }

    private void calculateAverage() {
        x = allX.stream()
                .mapToDouble(d -> d)
                .average()
                .orElse(0.0);
        y = allY.stream()
                .mapToDouble(d -> d)
                .average()
                .orElse(0.0);
        h = allH.stream()
                .mapToDouble(d -> d)
                .average()
                .orElse(0.0);
    }

    private void accountCamLocation() {
        x -= cameraX;
        y -= cameraY;
        h -= cameraH;
    }

    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(h));
    }

    public Pose2d startPos = new Pose2d(0,0, new Rotation2d(0));

    public void setStartPos(Pose2d ATag) {
        startPos = ATag;
    }

    public Pose2d getCorrectedPos(Pose2d ATag) {
        return new Pose2d(ATag.getX() - startPos.getX(), ATag.getY() - startPos.getY(), new Rotation2d(ATag.getHeading() - startPos.getHeading()));
    }
}
