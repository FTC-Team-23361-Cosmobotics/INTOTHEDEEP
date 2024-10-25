package org.firstinspires.ftc.teamcode.auton.pipeline;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SpikePosDetector implements VisionProcessor  {
    private static final Scalar BLUE = new Scalar(0, 0, 255);
    private static final Scalar GREEN = new Scalar(0, 255, 0);
    private static final Scalar RED = new Scalar(255, 0, 0);

    //640x480

    //center
    Point centerLeft = new Point(340, 290);
    Point centerRight = new Point(400, 330);

    //right
    Point rightLeft = new Point(75, 320);
    Point rightRight = new Point(175, 370);

    //ground color/shade
    Point groundLeft = new Point(310,440);
    Point groundRight = new Point(370,460);

    Mat centerSubmat = new Mat();
    Mat rightSubmat = new Mat();
    Mat groundSubmat = new Mat();

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();

    Scalar cMean;
    Scalar rMean;
    Scalar gMean;

    private final boolean useGround = true;

    private SPIKE_POS pos = SPIKE_POS.CENTER;

    private void inputToCb(Mat input) {
        // TODO: Only use one channel?
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }


    public SPIKE_POS getType() {
        return pos;
    }
    public Scalar getcMean(){
        return cMean;
    }
    public Scalar getrMean(){
        return rMean;
    }
    public double getGroundC(){
        return gMean.val[1];
    }

    public String debug() {
        return String.format("Pipeline debug:\nCenter: %s\nRight: %s\nGround: %s\nColor: %s", cMean, rMean, gMean, pos);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        inputToCb(frame);

        centerSubmat = frame.submat(new Rect(centerLeft, centerRight));
        rightSubmat = frame.submat(new Rect(rightLeft, rightRight));
        groundSubmat = frame.submat(new Rect(groundLeft,groundRight));


        cMean = new Scalar((int)Core.mean(centerSubmat).val[0], (int)Core.mean(centerSubmat).val[1], (int)Core.mean(centerSubmat).val[2]);
        rMean = new Scalar((int)Core.mean(rightSubmat).val[0], (int)Core.mean(rightSubmat).val[1], (int)Core.mean(rightSubmat).val[2]);
        gMean = new Scalar((int)Core.mean(groundSubmat).val[0], (int)Core.mean(groundSubmat).val[1], (int)Core.mean(groundSubmat).val[2]);

        double groundC = gMean.val[1]-30;

        double centerValue = cMean.val[1];
        double rightValue = rMean.val[1];

        boolean center;
        boolean right;

        if (useGround) {
            center  = centerValue < groundC - 15;
            right = rightValue < groundC - 15;
        } else {
            center  = centerValue < 180;
            right = rightValue < 180;
        }

        if(center && right){
            pos = (centerValue > rightValue) ? SPIKE_POS.CENTER : SPIKE_POS.LEFT;
            pos = (centerValue > rightValue) ? SPIKE_POS.CENTER : SPIKE_POS.LEFT;
        } else if (center){
            pos = SPIKE_POS.CENTER;
        } else if (right){
            pos = SPIKE_POS.LEFT;
        } else {
            pos = SPIKE_POS.RIGHT;
        }

        // draws rectangles where the submats are
        Imgproc.rectangle(frame, centerLeft, centerRight, center?GREEN:RED, 2);
        Imgproc.rectangle(frame, rightLeft, rightRight, right?GREEN:RED, 2);
        Imgproc.rectangle(frame, groundLeft, groundRight, BLUE, 2);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public enum SPIKE_POS {
        LEFT("LEFT", 0),
        CENTER("CENTER", 1),
        RIGHT("RIGHT", 2);

        private final String debug;
        private final int index;
        SPIKE_POS(String debug, int index) { this.debug = debug; this.index = index;}

        public String toString() { return debug; }
        public int index() { return index; }
    }
}