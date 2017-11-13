package edu.nctu.arlab.tango_sensors;

import android.annotation.SuppressLint;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.Rect;
import android.opengl.GLSurfaceView;
import android.os.Handler;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import android.widget.ToggleButton;

import com.google.atap.tangoservice.TangoCameraIntrinsics;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import java.io.ByteArrayOutputStream;
import java.nio.IntBuffer;
import java.util.concurrent.atomic.AtomicBoolean;

import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;

import static android.content.ContentValues.TAG;
import static com.google.atap.tangoservice.TangoCameraIntrinsics.TANGO_CAMERA_COLOR;
import static com.google.atap.tangoservice.TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE;

@SuppressWarnings("ConstantConditions")
class SensorTangoCamera {
    private final TangoSensors mTangoSensors;
    private ConnectedNode node;
    // OpenGL related stuff
    private final BackgroundRenderer backgroundRenderer;
    GLSurfaceView view;
    // Readings
    private int sequenceNumber        = 0;
    int cameraFrameCounter            = 0;
    private final static int minSleep = 20;
    //Parameters
    private final static int COMPRESSION_RAW  = 0;
    private final static int COMPRESSION_JPEG = 1;

    private int cameraType = TANGO_CAMERA_COLOR;

    private int     sleep       = 0;
    boolean enabled             = false;

    private final static int compression = COMPRESSION_JPEG;
    private int     quality     = 80;
    private final Rect    ROI         = new Rect();

    private boolean uEnabled    = false;
    private boolean uSleep      = false;
    private boolean uResolution = false;   // Update resolution. Special for camera case.
    private boolean uNewBitmap  = false;   // Request new bitmap creation.

    private String mTopicName = "";
    private String mTag       = "";
    // Camera calibration, should be obtained through the Tango API.

    private int maxCamWidth  = 1280;
    private int maxCamHeight = 720;

    private int camWidth  = 0;
    private int camHeight = 0;
    private int tangoCamWidth  = 0;
    private int tangoCamHeight = 0;
    private double  cx = 1.0;                               // Principal point
    private double  cy = 1.0;
    private double  fx = 1.0;                               // Focal Length
    private double  fy = 1.0;
    private double[]  _D     = { 1.0, 1.0, 1.0, 1.0, 1.0};  // Distortion
    private final double[]  _K     = {  fx, 0.0,  cx,             // Intrinsic camera matrix
                                0.0,  fy,  cy,
                                0.0, 0.0, 1.0 };
    private final double[]  _R     = { 1.0, 0.0, 0.0,             // Rectification matrix, used for stereo
                                0.0, 1.0, 0.0,             // cameras only, so omitted here.
                                0.0, 0.0, 1.0};
    private final double[]  _P     = { 1.0, 0.0, 1.0, 1.0,        // Projection matrix
                                0.0, 1.0, 1.0, 1.0,
                                0.0, 0.0, 1.0, 1.0};

    private Publisher<CompressedImage>   mCameraImagePublisher; // Compressed image publisher.
    private Publisher<CameraInfo>        mCameraInfoPublisher;  // Camera information publisher.

    private std_msgs.Header              mCameraImageHeader;
    private sensor_msgs.CompressedImage  mCameraImage;
    private Bitmap                       mBmp;
    private ByteArrayOutputStream        bStream;
    private ChannelBufferOutputStream    cStream;

    private sensor_msgs.CameraInfo       mCameraInfo;
    private std_msgs.Header              mCameraInfoHeader;
    private sensor_msgs.RegionOfInterest mCameraInfoROI;
    // Flags
    private final AtomicBoolean changed     = new AtomicBoolean(false);
    private final AtomicBoolean canSend     = new AtomicBoolean(true);
    private boolean nodeStarted       = false;

    private TextView     textViewFPS;
    private TextView     textViewCompression;
    private TextView     textViewResolution;
    private TextView     textViewROI;
    private EditText     editTextSleep;
    private EditText     editTextQuality;
    private EditText     editTextResolutionWidth;
    private EditText     editTextResolutionHeight;
    private ToggleButton toggleButtonEnabled;
    private ImageView    imageViewIndicator;
    @SuppressWarnings({"unused", "FieldCanBeLocal"})
    private Spinner      spinnerCompression;
    private Button       buttonApply;

    private class DataSender2 implements Runnable {
        public void run(){
            if (!nodeStarted) return;

            try {
                if ((mCameraImagePublisher != null) && (backgroundRenderer.mPixels != null) &&
                        (backgroundRenderer.frameAvailable.get())) {
                    backgroundRenderer.frameAvailable.set(false);
                    boolean skipCycle = false;

                    // Publish camera info, change this later from /tango_origin to correct values!!!
                    // Needs optimization.
                    mCameraInfoHeader.setSeq(sequenceNumber);
                    if (cameraType == TANGO_CAMERA_COLOR)
                        mCameraInfoHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mColorCameraTF);
                    if (cameraType == TANGO_CAMERA_FISHEYE)
                        mCameraInfoHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mFisheyeCameraTF);

                    mCameraInfoHeader.setStamp(node.getCurrentTime());

                    mCameraInfo.setWidth(camWidth);
                    mCameraInfo.setHeight(camHeight);

                    mCameraInfoROI.setWidth(ROI.width());
                    mCameraInfoROI.setHeight(ROI.height());
                    mCameraInfoROI.setXOffset(ROI.left);
                    mCameraInfoROI.setYOffset(ROI.top);
                    if ((ROI.left !=0 ) || (ROI.top != 0) ||
                            (ROI.right != camWidth) || (ROI.bottom != camHeight)) {
                        mCameraInfoROI.setDoRectify(false);
                    }
                    else
                    {
                        mCameraInfoROI.setDoRectify(false);
                    }

                    mCameraInfoPublisher.publish(mCameraInfo);

                    // Camera image
                    mCameraImageHeader.setSeq(sequenceNumber);
                    mCameraImageHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mBaseLinkTF);
                    mCameraImageHeader.setStamp(node.getCurrentTime());
                    mCameraImage.setHeader(mCameraImageHeader);

                    // Create new bitmap if was requested.
                    if (uNewBitmap) {
                        mBmp = Bitmap.createBitmap(backgroundRenderer.offscreenSize_.x, backgroundRenderer.offscreenSize_.y, Bitmap.Config.ARGB_8888);
                        uNewBitmap = false;
                    }
                    // Copy image from backgroundRenderer
                    while (backgroundRenderer.blockImage.get()) {
                        Thread.sleep(1);
                    }
                    backgroundRenderer.blockImage.set(true);

                    try {
                        mBmp.copyPixelsFromBuffer(IntBuffer.wrap(backgroundRenderer.mPixels));
                    } catch (Exception e) {
                        Log.i(mTag, "Skipping cycle due to exception:",e);
                        skipCycle  = true;
                        uNewBitmap = true;
                    }
                    backgroundRenderer.blockImage.set(false);

                    if (!skipCycle) {

                        // Compress bitmap
                        mBmp.compress(Bitmap.CompressFormat.JPEG, quality, bStream);

                        //convertedBitmap.compress(Bitmap.CompressFormat.JPEG, 10, bStream);
                        // Convert result to byte array
                        try {
                            cStream.write(bStream.toByteArray());
                        } catch (Exception e) {
                            Log.e(TAG,"Exception", e);
                        }

                        mCameraImage.setFormat("jpeg");
                        mCameraImage.setData(cStream.buffer().copy());

                        try {
                            cStream.buffer().clear();
                            bStream.reset();
                        } catch (Exception e) {
                            Log.e(TAG,"Exception", e);
                        }
                        mCameraImagePublisher.publish(mCameraImage);

                        sequenceNumber++;
                    }
                }

                // I have no idea why this is happening. The order should be like this, otherwise
                // it will stall the transmission.
                if (uResolution) {
                    Log.i(mTag,"RESOLUTION: "+String.valueOf(camWidth)+"x"+String.valueOf(camHeight));
                    backgroundRenderer.changeResolution(camWidth, camHeight);
                    //while (backgroundRenderer.changeOffscreen.get() == true) { Thread.sleep(1); };
                    uNewBitmap  = true;
                    uResolution = false;
                }

                backgroundRenderer.saveFrame();
            } catch (Exception e) {
                Log.e(TAG, "exception", e);
            }

            changed.set(false);
        }
    }

    SensorTangoCamera(TangoSensors pTangoSensors, int cameraID){
        mTangoSensors = pTangoSensors;

        backgroundRenderer = new BackgroundRenderer(mTangoSensors);
        backgroundRenderer.cameraType = cameraID;
        cameraType = cameraID;

        if (cameraID == TANGO_CAMERA_COLOR) {
            mTopicName = "tango_camera_color";
            mTag       = "TANGO CAMERA: COLOR";

            maxCamWidth  = 1280;
            maxCamHeight = 720;

            view = (GLSurfaceView) mTangoSensors.findViewById(R.id.surfaceViewCameraColor);
            textViewFPS = (TextView) mTangoSensors.findViewById(R.id.textViewColorCameraFPS);
            textViewCompression = (TextView) mTangoSensors.findViewById(R.id.textViewColorCameraCompression);
            textViewResolution = (TextView) mTangoSensors.findViewById(R.id.textViewColorCameraRes);
            textViewROI = (TextView) mTangoSensors.findViewById(R.id.textViewColorCameraROI);
            editTextSleep = (EditText) mTangoSensors.findViewById(R.id.editTextColorCameraSleep);
            editTextQuality = (EditText) mTangoSensors.findViewById(R.id.editTextColorCameraQuality);
            imageViewIndicator = (ImageView) mTangoSensors.findViewById(R.id.imageViewColorCameraIndicator);
            toggleButtonEnabled = (ToggleButton) mTangoSensors.findViewById(R.id.toggleButtonColorCameraEnabled);
            spinnerCompression = (Spinner) mTangoSensors.findViewById(R.id.spinnerColorCameraCompression);
            editTextResolutionWidth = (EditText) mTangoSensors.findViewById(R.id.editTextColorCameraWidth);
            editTextResolutionHeight = (EditText) mTangoSensors.findViewById(R.id.editTextColorCameraHeight);
            buttonApply = (Button) mTangoSensors.findViewById(R.id.buttonColorCameraApply);
        }
        if (cameraID == TANGO_CAMERA_FISHEYE) {
            mTopicName = "tango_camera_fisheye";
            mTag       = "TANGO CAMERA: FISHEYE";

            maxCamWidth  = 640;
            maxCamHeight = 480;

            view = (GLSurfaceView) mTangoSensors.findViewById(R.id.surfaceViewCameraFisheye);
            textViewFPS = (TextView) mTangoSensors.findViewById(R.id.textViewFisheyeCameraFPS);
            textViewCompression = (TextView) mTangoSensors.findViewById(R.id.textViewFisheyeCameraCompression);
            textViewResolution = (TextView) mTangoSensors.findViewById(R.id.textViewFisheyeCameraRes);
            textViewROI = (TextView) mTangoSensors.findViewById(R.id.textViewFisheyeCameraROI);
            editTextSleep = (EditText) mTangoSensors.findViewById(R.id.editTextFisheyeCameraSleep);
            editTextQuality = (EditText) mTangoSensors.findViewById(R.id.editTextFisheyeCameraQuality);
            imageViewIndicator = (ImageView) mTangoSensors.findViewById(R.id.imageViewFisheyeCameraIndicator);
            toggleButtonEnabled = (ToggleButton) mTangoSensors.findViewById(R.id.toggleButtonFisheyeCameraEnabled);
            spinnerCompression = (Spinner) mTangoSensors.findViewById(R.id.spinnerFisheyeCameraCompression);
            editTextResolutionWidth = (EditText) mTangoSensors.findViewById(R.id.editTextFisheyeCameraWidth);
            editTextResolutionHeight = (EditText) mTangoSensors.findViewById(R.id.editTextFisheyeCameraHeight);
            buttonApply = (Button) mTangoSensors.findViewById(R.id.buttonFisheyeCameraApply);
        }

        ROI.left   = 0;
        ROI.top    = 0;
        ROI.right  = maxCamWidth;
        ROI.bottom = maxCamHeight;

        toggleButtonEnabled.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    imageViewIndicator.setImageResource(R.drawable.circle_green);
                    startSensor();
                    prepareNode();
                    executeNode();
                }
                else
                {
                    imageViewIndicator.setImageResource(R.drawable.circle_red);
                    stopSensor();
                    shutdownNode();
                }

                if (mTangoSensors.updatePreferences) {
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putBoolean("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/"+"enabled", enabled);
                    editor.apply();
                }
                if (mTangoSensors.updateServerParams) {
                    uEnabled = true;
                }
            }
        });

        editTextSleep.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    sleep = Integer.parseInt(s.toString());
                } catch (Exception e) {
                    sleep = 0;
                }

                if (mTangoSensors.updatePreferences) {
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/"+"sleep", sleep);
                    editor.apply();
                }

                if (mTangoSensors.updateServerParams) {
                    uSleep = true;
                }
            }

            @Override
            public void afterTextChanged(Editable s) {
                if (editTextSleep.length() == 0) {
                    editTextSleep.setText("0");
                    editTextSleep.setSelection(1);
                }
            }
        });

        buttonApply.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sleep = Integer.valueOf(editTextSleep.getText().toString());
                if (sleep < 0)   sleep = 0;

                quality = Integer.valueOf(editTextQuality.getText().toString());
                if (quality<0)   quality = 0;
                if (quality>100) quality = 100;

                camWidth = Integer.valueOf(editTextResolutionWidth.getText().toString());
                if (camWidth < 1) camWidth = 1;
                if (camWidth > maxCamWidth) camWidth = maxCamWidth;

                camHeight = Integer.valueOf(editTextResolutionHeight.getText().toString());
                if (camHeight < 1) camHeight = 1;
                if (camHeight > maxCamHeight) camHeight = maxCamHeight;

                ROI.top    = 0;
                ROI.left   = 0;
                ROI.right  = camWidth;
                ROI.bottom = camHeight;

                updatePreferences();
                updateReadings();
                forceUpdateParams();
                updateEdits();

                uResolution = true;

                mTangoSensors.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        if (backgroundRenderer.cameraType == TANGO_CAMERA_COLOR) {
                            Toast toast = Toast.makeText(mTangoSensors, "Color camera settings updated", Toast.LENGTH_SHORT);
                            toast.show();
                        }
                        else
                        if (backgroundRenderer.cameraType == TANGO_CAMERA_FISHEYE) {
                            Toast toast = Toast.makeText(mTangoSensors, "Fisheye camera settings updated", Toast.LENGTH_SHORT);
                            toast.show();
                        }
                    }
                });
            }
        });


        view.setEGLContextClientVersion(2);
        view.setDebugFlags(GLSurfaceView.DEBUG_CHECK_GL_ERROR);
        view.setRenderer(backgroundRenderer);
        view.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
    }

    @SuppressLint("SetTextI18n")
    void updateFPS() {
        textViewFPS.setText("FPS: "+String.valueOf(cameraFrameCounter));
        cameraFrameCounter = 0;
    }

    //------------------------------------ Start Sensor ------------------------------------------//
    private void startSensor() {
        Log.i(mTag, "startSensor()");

        if (mTangoSensors.tangoStarted) mTangoSensors.stopTango();
        mTangoSensors.startTango();
        enabled = true;
    }

    //------------------------------------ Stop Sensor -------------------------------------------//
    private void stopSensor() {
        Log.i(mTag, "stopSensor()");

        if (mTangoSensors.tangoStarted) mTangoSensors.stopTango();

        enabled = false;
        mTangoSensors.startTango(); //Tango should be restarted since other sensors still might use it.
    }

    //----------------------------------- Load Preferences ---------------------------------------//

    void loadPreferences(){
        sleep      = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/"+"sleep",       sleep);
        enabled    = mTangoSensors.prefs.getBoolean("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/"+"enabled", enabled);
        quality    = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/"+"quality",     quality);
        camWidth   = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/width",          camWidth);
        camHeight  = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/height",         camHeight);
        ROI.left   = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/left",       ROI.left);
        ROI.top    = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/top",        ROI.top);
        ROI.right  = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/right",      ROI.right);
        ROI.bottom = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/bottom",     ROI.bottom);

        if (backgroundRenderer.cameraType == TANGO_CAMERA_COLOR)
        Log.i(mTag,"RESOLUTION L:"+String.valueOf(camWidth)+'x'+String.valueOf(camHeight));

        updateReadings();
        updateEdits();

        uResolution = true;
    }

    //------------------------------------ Update Preferences ------------------------------------//
    private void updatePreferences() {
        if (mTangoSensors.updatePreferences) {
            SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
            editor.putBoolean("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/enabled", enabled);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/sleep",       sleep);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/quality",     quality);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/width",       camWidth);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/height",      camHeight);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/left",    ROI.left);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/top",     ROI.top);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/right",   ROI.right);
            editor.putInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/roi/bottom",  ROI.bottom);
            editor.apply();
        }
    }

    void updateReadings() {
        String s;
        if (compression == COMPRESSION_RAW)  s = "RAW";
        else
        if (compression == COMPRESSION_JPEG) s = "JPEG";

        textViewCompression.setText(s+" ["+String.valueOf(quality)+"]");
        textViewResolution.setText("Res: "+ String.valueOf(camWidth)+"x"+String.valueOf(camHeight));
        textViewROI.setText("["+String.valueOf(ROI.left)+","+String.valueOf(ROI.top)+';'+String.valueOf(ROI.right)+","+String.valueOf(ROI.bottom)+"]");
    }

    //------------------------------------- Update Edits -----------------------------------------//
    void updateEdits(){
        editTextQuality.setText(String.valueOf(quality));
        editTextSleep.setText(String.valueOf(sleep));
        editTextResolutionWidth.setText(String.valueOf(camWidth));
        editTextResolutionHeight.setText(String.valueOf(camHeight));
        toggleButtonEnabled.setChecked(enabled);
    }

    //------------------------------------ Camera parameters -------------------------------------//
    private void getCameraParameters(){
        if (mTangoSensors.mTango == null) return;

        TangoCameraIntrinsics ccIntrinsics;
        ccIntrinsics = mTangoSensors.mTango.getCameraIntrinsics(backgroundRenderer.cameraType);
        tangoCamHeight = ccIntrinsics.height;
        tangoCamWidth  = ccIntrinsics.width;
        cx = ccIntrinsics.cx;
        cy = ccIntrinsics.cy;
        fx = ccIntrinsics.fx;
        fy = ccIntrinsics.fy;
        _K[0] =  fx; _K[1] = 0.0; _K[2] =  cx;
        _K[3] = 0.0; _K[4] =  fy; _K[5] =  cy;
        _K[6] = 0.0; _K[7] = 0.0; _K[8] = 1.0;
        _D = ccIntrinsics.distortion;
    }

    //---------------------------------------- ROS -----------------------------------------------//
    //--------------------------------------------------------------------------------------------//

    //----------------------------------ROS: Set Node --------------------------------------------//
    void setNode(ConnectedNode pNode) {
        this.node = pNode;
    }

    //--------------------------------- ROS: Prepare Node ----------------------------------------//
    void prepareNode(){
        if ((node == null) || (!enabled)) return;

        if (backgroundRenderer.cameraType == TANGO_CAMERA_COLOR) {
            mCameraImagePublisher = node.newPublisher("/tango_sensors/tango_camera_color/compressed" , sensor_msgs.CompressedImage._TYPE);
            mCameraInfoPublisher  = node.newPublisher("/tango_sensors/tango_camera_color/camera_info", sensor_msgs.CameraInfo._TYPE);
        }if (backgroundRenderer.cameraType == TANGO_CAMERA_FISHEYE) {
            mCameraImagePublisher = node.newPublisher("/tango_sensors/tango_camera_fisheye/compressed", sensor_msgs.CompressedImage._TYPE);
            mCameraInfoPublisher  = node.newPublisher("/tango_sensors/tango_camera_fisheye/camera_info", sensor_msgs.CameraInfo._TYPE);
        }

        mCameraImageHeader = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        mCameraImage       = node.getTopicMessageFactory().newFromType(sensor_msgs.CompressedImage._TYPE);
        //mBmp               = Bitmap.createBitmap(backgroundRenderer.offscreenSize_.x, backgroundRenderer.offscreenSize_.y, Bitmap.Config.ARGB_8888);
        uNewBitmap         = true; //Bitmap will be created on the first run.
        bStream            = new ByteArrayOutputStream();
        cStream            = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());

        mCameraInfo        = node.getTopicMessageFactory().newFromType(sensor_msgs.CameraInfo._TYPE);
        mCameraInfoHeader  = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        mCameraInfoROI     = node.getTopicMessageFactory().newFromType(sensor_msgs.RegionOfInterest._TYPE);

        getCameraParameters();

        mCameraInfoROI.setWidth(tangoCamWidth);
        mCameraInfoROI.setHeight(tangoCamHeight);
        mCameraInfoROI.setXOffset(0);
        mCameraInfoROI.setYOffset(0);
        mCameraInfoROI.setDoRectify(false);

        mCameraInfo.setHeader(mCameraInfoHeader);
        mCameraInfo.setWidth(camWidth);
        mCameraInfo.setHeight(camHeight);
        if (backgroundRenderer.cameraType == TANGO_CAMERA_COLOR)   mCameraInfo.setDistortionModel("rational_polynomial");
        else
        if (backgroundRenderer.cameraType == TANGO_CAMERA_FISHEYE) mCameraInfo.setDistortionModel("equidistant"); // Note: Not supported by ROS.
        mCameraInfo.setD(_D);
        mCameraInfo.setR(_R);
        mCameraInfo.setP(_P);
        mCameraInfo.setK(_K);
        mCameraInfo.setBinningX(tangoCamWidth  / backgroundRenderer.offscreenSize_.x);
        mCameraInfo.setBinningY(tangoCamHeight / backgroundRenderer.offscreenSize_.y);
        mCameraInfo.setRoi(mCameraInfoROI);

        sequenceNumber = 0;
    }

    //--------------------------------- ROS: Shutdown Node ---------------------------------------//
    void shutdownNode() {
        nodeStarted = false;
        sequenceNumber = 0;
        if (mCameraImagePublisher != null) mCameraImagePublisher.shutdown();
        if (mCameraInfoPublisher  != null) mCameraInfoPublisher.shutdown();
    }

    //--------------------------------- ROS: Execute Node ----------------------------------------//
    void executeNode(){
        if ((node == null) || (!enabled)) return;

        nodeStarted = true;
    }

    void sendData(){
        if (!nodeStarted) return;
        if ( (sleep==0) || ( (sleep>0) && (canSend.get() )))  {
            if (sleep > 0) canSend.set(false);

            new Thread(new DataSender2()).start();

            changed.set(false);

            if (sleep > 0) {
                int iSleep = (sleep < minSleep) ? minSleep : sleep;
                // Setting delay after sleep
                final Handler handler = new Handler();
                handler.postDelayed(new Runnable() {
                    @Override
                    public void run() {
                        canSend.set(true);
                    }
                }, iSleep);
            }
        }
    }

    //--------------------------------- ROS: Update Parameters -----------------------------------//
    void updateParams() {
        if (node == null) return;

        if (uEnabled) {
            uEnabled = false;
            ParameterTree params = node.getParameterTree();
            params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/enabled"), enabled);
        }
        if (uSleep) {
            uSleep = false;
            ParameterTree params = node.getParameterTree();
            params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/sleep"), sleep);
        }
    }
    //---------------------------- ROS: Force Update Params, immediately -------------------------//
    private void forceUpdateParams() {
        if (node == null) return;

        ParameterTree params = node.getParameterTree();
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/enabled"),    enabled);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/sleep"),      sleep);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/quality"),    quality);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/width"),      camWidth);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/height"),     camHeight);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/roi/top"),    ROI.top);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/roi/left"),   ROI.left);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/roi/bottom"), ROI.bottom);
        params.set(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName + "/roi/right"),  ROI.right);
    }

    //----------------------------ROS: Load server parameters ------------------------------------//
    void loadServerParams() {
        if (node == null) return;

        ParameterTree params = node.getParameterTree();
        try { sleep      = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/sleep"),      sleep);     }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { enabled    = params.getBoolean(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/enabled"),    enabled);   }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { quality    = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/quality"),    quality);   }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { camWidth   = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/width"),      camWidth);  }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { camHeight  = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/height"),     camHeight); }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { ROI.top    = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/roi/top"),    ROI.top); }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { ROI.left   = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/roi/left"),   ROI.left); }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { ROI.right  = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/roi/right"),  ROI.right); }catch (Exception e) {Log.e(TAG,"Exception", e);}
        try { ROI.bottom = params.getInteger(GraphName.of("/" + mTangoSensors.mNodeNamespace + "/" + mTopicName +"/roi/bottom"), ROI.bottom); }catch (Exception e) {Log.e(TAG,"Exception", e);}

        // Failsafe, probably should be a separate function.
        if (sleep<0)                  sleep     = 0;
        if (quality<0)                quality   = 0;
        if (quality>100)              quality   = 100;
        if (camWidth < 1)             camWidth  = 320;
        if (camWidth > maxCamWidth)   camWidth  = maxCamWidth;
        if (camHeight < 1)            camHeight = 240;
        if (camHeight > maxCamHeight) camHeight = maxCamHeight;

        ROI.top    = 0;
        ROI.left   = 0;
        ROI.right  = camWidth;
        ROI.bottom = camHeight;

        uResolution = true;
    }
}
