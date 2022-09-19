package org.firstinspires.ftc.teamcode.drive;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TapeMeasurePipelineAveragingContoursBlue extends OpenCvPipeline {

    Mat hsvMat = new Mat();
    Mat filteredHSV = new Mat();
    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    final MatOfInt hull = new MatOfInt();
    List<MatOfPoint> outputContours = new ArrayList<>();

    Rect[] boundingBoxes;

    float totalxPositions;
    float totalyPositions;

    float xPosition;
    float yPosition;

    @Override
    public Mat processFrame(Mat input){
        //copy input from camera into the matrix
        //input.copyTo(matrix);

        if(input.empty()){
            return input;
        }
        //convert to hsv
        //Imgproc.cvtColor(matrix,matrix,Imgproc.COLOR_RGB2HSV);
        //convert to YcrCb
        //Mat hsvMat = new Mat();
        Imgproc.cvtColor(input,hsvMat,Imgproc.COLOR_RGB2HSV);
        //filter in case camera isnt working


        //make a hsv threshold
        // apparently hue is half the real value so we may need to divide these by 2 for h
        Scalar lowHSV = new Scalar(21,241,157);
        Scalar highHSV = new Scalar(54,255,184);

        //make a ycrcb threshold
        //Scalar lowYCrCb = new Scalar();
        //Scalar highYCrCb = new Scalar();

        //filter hsv matrix through hsv threshold
        //Mat filteredHSV = new Mat();
        Core.inRange(hsvMat,lowHSV,highHSV,filteredHSV);
        //Core.inRange(matrix,lowYCrCb,highYCrCb,matrix);

        //find countours

        //List<MatOfPoint> contours = new ArrayList<>();
        //Mat hierarchy = new Mat();

        contours.clear();
        int mode = Imgproc.RETR_LIST;
        int method = Imgproc.CHAIN_APPROX_SIMPLE;

        Imgproc.findContours(filteredHSV,contours,hierarchy,mode,method);


        /*List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(matrix,contours,hierarchy,Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE); */

        //FILTER CONTOURS

        //set parameters to filter out
        double minArea = 0.0;//500//100 also well
        double minPerimeter = 0;
        double minWidth = 0;
        double maxWidth = 100000000;
        double minHeight = 0;
        double maxHeight = 1000000000;
        double[] solidity = {0, 100};
        double maxVertexCount = 1000000;
        double minVertexCount = 0;
        double minRatio = 0;
        double maxRatio = 1000;

        // create lists to store data
        //final MatOfInt hull = new MatOfInt();
        //outputContours = new ArrayList<>();


        outputContours.clear();

        //put contours that match into the output list
        for (int i = 0; i < contours.size(); i++) {
            final MatOfPoint contour = contours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            outputContours.add(contour);
        }

        //draw bounding boxes around the contours we filtered
        // output is the list of recognitions/ contours we accepted
        //matrix is the current matrix we're using which was passed through the hsv threshold
        // outputContours list has the list of contours you want/ equivalent to the recognitions list in ML

        final Scalar Fill = new Scalar(0.5,76.9,89.8); //fill colour of drawing is green

        boundingBoxes = new Rect[outputContours.size()];

        for(int i = 0; i < outputContours.size(); i++){
            boundingBoxes[i] = Imgproc.boundingRect(outputContours.get(i));
            if(boundingBoxes[i]!= null) {
                Imgproc.rectangle(hsvMat, boundingBoxes[i], Fill);

            }
        }

        List<MatOfPoint> targets = outputContours;
        float [] xPositions = new float[targets.size()];
        float [] yPositions = new float[targets.size()];

        int i = 0;
        if(targets != null){
            for(MatOfPoint target: targets){
                float xlocation = Imgproc.boundingRect(target).x + (Imgproc.boundingRect(target).width/2);
                float ylocation = Imgproc.boundingRect(target).y + (Imgproc.boundingRect(target).height/2);
                xPositions[i] = xlocation;
                yPositions[i] = ylocation;
                i++;

            }
        }

        totalxPositions=0;
        totalyPositions=0;
        xPosition = 0;
        yPosition = 0;


        if(xPositions.length != 0) {
            for (int p = 0; p < xPositions.length; p++) {
                totalxPositions = totalxPositions+xPositions[p];
                totalyPositions = totalyPositions+yPositions[p];
            }
            xPosition = totalxPositions/xPositions.length;
            yPosition = totalyPositions/yPositions.length;
        }

        return filteredHSV;
    }

    public float getTargetXPos(){
        return xPosition;
    }
    public float getTargetYPos(){
        return yPosition;
    }

}
