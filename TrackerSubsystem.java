package teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class TrackerSubsystem {
    //declare hardware in the subsystem
    public HardwareMap hardwareMap;
    public WebcamName webcamName;
    public Servo trackingServo;
    public OpMode opMode;
    public Telemetry telemetry;

    //declare tensorflow variables
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    //declare vuforia info
    private static final String VUFORIA_KEY =
            "ARxx8iD/////AAABmbS2nmcBoEChkoWzG9AGa2EDVhPKPlhaEz6XN4C2VcvZroE1t5+3F8ZoVL77a7Ynof/UlFJ4thqxL+u5pLag9/4VX4JDfVl3S8EEAsvBdcvssq6epPgQ2vDaeFb6hMa2qNqCfkdVY0TnvErz0XWAIzimTJISvctyRtS2JCf3nJQ3FBqNuw6h5mmM5y09NdDEH4oNvN28jQbxkSfgK0BejMp/ElJGU4vawIS9XyOD9i7rBrrUZTyqGnmURDItpOwRNseOtNjtXGIKszEHTLh6q5L5pl/kBybGHi40G2CjomR50XvwU988t4eQ4IXAEcjliYhkLjx3xq1VCfdayZBB0mNP18T7gbW/btNWkLrzXQFk";

    //declare vuforia and tensorflow objects
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    //set PID variables
    public float error = 0;
    double kP = 0.1;
    double kD = 0.35;//0.3;
    double kI= 0.0;

    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    double integral = 0;
    double lastError = 0;
    float pixelsToServoMv = (float) 0.000666;

    //set camera tracking vars
    double servoPosition = 0.5;
    float centeredLine = 310;

    // write constructor
    public TrackerSubsystem(HardwareMap hwMap, OpMode oMode) {

        //set hardware map
        hardwareMap = hwMap;
        this.opMode = oMode;
        this.telemetry = opMode.telemetry;

        // set mech components
        webcamName = hardwareMap.get(WebcamName.class, "AndreWebcam");
        trackingServo = hardwareMap.get(Servo.class, "TrackingServo");

        //set up servo
        trackingServo.setDirection(Servo.Direction.FORWARD);
        trackingServo.scaleRange(0,1);

        //set up tensorflow
        initVuforia();
        initTfod();

        trackingServo.setPosition(servoPosition);

        if(tfod !=null) {
            tfod.activate();
            tfod.setZoom(2.5,16.0/9.0);
        }

        FtcDashboard.getInstance().startCameraStream(this.tfod, 0);

    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "AndreWebcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    public void CenterCamera(){

        if (tfod !=null){
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if(updatedRecognitions != null) {

                int i = 0;
                for( Recognition recognition: updatedRecognitions) {
                    //find center of curr object
                    float leftSide = recognition.getLeft();
                    float rightSide = recognition.getRight();
                    float middleObject = (leftSide+rightSide)/2;

                    //set error from center
                    double servoSpeed = PID(centeredLine , middleObject);


                    servoPosition = servoPosition - (servoSpeed);

                    trackingServo.setPosition(servoPosition);
                }
            }
        }

    }
    public double PID (float setpoint ,float feedback){
        error = (setpoint-feedback)*pixelsToServoMv;
        integral = integral +(error *timer.time());


        double derivative = (error -lastError)/ timer.time();
        lastError= error;
        timer.reset();
        double adjustment = kP*error + kD*derivative + kI*integral;

        this.telemetry.addData("adjustment", adjustment);
        this.telemetry.update();
        return adjustment;
    }

}
