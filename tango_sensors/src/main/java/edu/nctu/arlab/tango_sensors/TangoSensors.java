package edu.nctu.arlab.tango_sensors;

import org.ros.android.RosActivity;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;

import android.annotation.SuppressLint;
import android.app.ActionBar;
import android.app.ActivityManager;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Point;
import android.os.Bundle;
import android.os.Debug;
import android.os.Handler;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;

import std_msgs.Empty;

import static android.content.ContentValues.TAG;
import static android.os.SystemClock.sleep;

public class TangoSensors extends RosActivity implements NodeMain {
    public final String mNodeNamespace   = "tango_sensors";

    public String mWorldTF         = "map";
    public String mOdomTF          = "odom";
    public String mBaseLinkTF      = "base_link";
    public String mColorCameraTF   = "color_camera";
    public String mFisheyeCameraTF = "fisheye_camera";

    private final int updateFrequency = 200;
    public final boolean updatePreferences = true;
    public final boolean updateServerParams = true;

    private volatile boolean updateEditsMutex = false; // When this one is set to "true", edits will update once.
    // TANGO
    volatile public boolean tangoStarted = false;
    private volatile boolean tangoStartPermission = false; // Until this is true, startTango will have no effect.
    // Done so to prevent multiple Tango start/stops on launch

    private ConnectedNode            mNode;

    // Sensors
    private SensorTangoPose mTangoPoseSensor;
    private SensorTangoPCL mPointCloudSensor;
    private SensorTangoCamera mColorCamera;
    private SensorTangoCamera mFisheyeCamera;
    // Transforms
    private TransformBroadcaster mTransformBroadcaster;

    // Inner TANGO:Pose transform quaternion.
    // The TF2 transform cannot be used due to messing with translation coordinates as well.
    private static final float innerPoseOX = (float) (-Math.sin(Math.PI / 4.0)); //-0.707f
    private static final float innerPoseOY = 0.000f;                              // 0.000f
    private static final float innerPoseOZ = (float) (+Math.sin(Math.PI / 4.0)); //+0.707f
    private static final float innerPoseOW = 0.000f;                              // 0.000f

    //TF2 Transform "/tango_point_cloud"
    volatile public float tf2PTX = 0.00f;
    volatile public float tf2PTY = 0.00f;
    volatile public float tf2PTZ = 0.00f;
    volatile public float tf2PRX = 0.50f;
    volatile public float tf2PRY = 0.50f;
    volatile public float tf2PRZ = 0.50f;
    volatile public float tf2PRW = 0.50f;

    private EditText editTextPTX;
    private EditText editTextPTY;
    private EditText editTextPTZ;
    private EditText editTextPRX;
    private EditText editTextPRY;
    private EditText editTextPRZ;
    private EditText editTextPRW;

    private TextView textViewMemory;
    private TextView textViewRosTime;

    public SharedPreferences prefs;

    //TANGO
    public Tango       mTango;
    private TangoConfig mConfig;

    //OpenGL
    private HashMap<Integer, Integer> cameraTextures_;

    // Memory
    private int  totalMem;
    private int  totalMemLH;
    private long totalMemMX;

    //onCreate
    @SuppressLint("UseSparseArrays")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.d("TANGO_SENSORS:", "onCreate");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        // Inflate action bar with some elements
        ActionBar mActionBar = getActionBar();

        if (mActionBar != null) {
            mActionBar.setDisplayShowHomeEnabled(false);
            mActionBar.setDisplayShowTitleEnabled(false);
        }
        LayoutInflater mInflater = LayoutInflater.from(this);

        @SuppressLint("InflateParams") View mCustomView = mInflater.inflate(R.layout.custom_action_bar, null);

        if (mActionBar != null) {
            mActionBar.setCustomView(mCustomView);
            mActionBar.setDisplayShowCustomEnabled(true);
        }

        textViewMemory  = (TextView) findViewById(R.id.textViewMemory);
        textViewRosTime = (TextView) findViewById(R.id.textViewRosTime);

        // Transform
        mTransformBroadcaster = new TransformBroadcaster(this);

        // Sensors
        mTangoPoseSensor = new SensorTangoPose(this, mTransformBroadcaster);
        mPointCloudSensor = new SensorTangoPCL(this, mTransformBroadcaster);
        mColorCamera = new SensorTangoCamera(this, TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
        mFisheyeCamera = new SensorTangoCamera(this, TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE);

        // Better not to change this, since this comes from another code, and is dealing with
        // OpenGL to capture, process and publish image messages to ROS,  which was A HELL
        // to implement to begin.
        cameraTextures_ = new HashMap<Integer, Integer>();

        //----------------------------------------------------------------------------------------//

        //TF2 Translation (/tango_pose)
        editTextPTX = (EditText) findViewById(R.id.editTextPTX);
        editTextPTX.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2PTX = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("/tango_pose/translation/x", tf2PTX);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextPTY = (EditText) findViewById(R.id.editTextPTY);
        editTextPTY.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2PTY = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("/tango_pose/translation/y", tf2PTY);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextPTZ = (EditText) findViewById(R.id.editTextPTZ);
        editTextPTZ.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2PTZ = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("/tango_pose/translation/z", tf2PTZ);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextPRX = (EditText) findViewById(R.id.editTextPRX);
        editTextPRX.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2PRX = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("/tango_pose/rotation/x", tf2PRX);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextPRY = (EditText) findViewById(R.id.editTextPRY);
        editTextPRY.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2PRY = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("/tango_pose/rotation/y", tf2PRY);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextPRZ = (EditText) findViewById(R.id.editTextPRZ);
        editTextPRZ.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2PRZ = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("/tango_pose/rotation/z", tf2PRZ);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextPRW = (EditText) findViewById(R.id.editTextPRW);
        editTextPRW.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2PRW = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putFloat("/tango_pose/rotation/w", tf2PRW);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });
        //                                   TRANSFORM EDITS                                      //
        EditText editTextWorldTF = (EditText) findViewById(R.id.editTextWorldTF);
        editTextWorldTF.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    mWorldTF = s.toString();
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putString("/tf/world", mWorldTF);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        EditText editTextOdomTF = (EditText) findViewById(R.id.editTextOdomTF);
        editTextOdomTF.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    mOdomTF = s.toString();
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putString("/tf/odom", mOdomTF);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        EditText editTextBaseLinkTF = (EditText) findViewById(R.id.editTextBaseLinkTF);
        editTextBaseLinkTF.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    mBaseLinkTF = s.toString();
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putString("/tf/base_link", mBaseLinkTF);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        EditText editTextColorCameraTF = (EditText) findViewById(R.id.editTextTangoColorCamTF);
        editTextColorCameraTF.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    mColorCameraTF = s.toString();
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putString("/tf/color_camera", mColorCameraTF);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        EditText editTextFisheyeCameraTF = (EditText) findViewById(R.id.editTextTangoFisheyeCamTF);
        editTextFisheyeCameraTF.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    mFisheyeCameraTF = s.toString();
                    SharedPreferences.Editor editor = prefs.edit();
                    editor.putString("/tf/fisheye_camera", mFisheyeCameraTF);
                    editor.apply();
                } catch (Exception e) {
                    Log.i(TAG,"Exception", e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        //----------------------------------------------------------------------------------------//
        CheckBox checkBoxKeepAwake = (CheckBox) findViewById(R.id.checkBoxKeepAwake);
        checkBoxKeepAwake.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                SharedPreferences.Editor editor = prefs.edit();

                if (isChecked){
                    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

                    editor.putBoolean("/keep_awake", true);
                }
                else
                {
                    getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

                    editor.putBoolean("/keep_awake", false);
                }
                
                editor.apply();
            }
        });

        Button buttonTF1Default = (Button) findViewById(R.id.buttonTF1Default);
        buttonTF1Default.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mTransformBroadcaster.editTextTTX.setText("0.0");
                mTransformBroadcaster.editTextTTY.setText("0.0");
                mTransformBroadcaster.editTextTTZ.setText("0.0");
                mTransformBroadcaster.editTextTRX.setText("0.0");
                mTransformBroadcaster.editTextTRY.setText("0.0");
                mTransformBroadcaster.editTextTRZ.setText("0.0");
                mTransformBroadcaster.editTextTRW.setText("1.0");

                Toast toast = Toast.makeText(v.getContext(), "Transformations for pose are reset.", Toast.LENGTH_SHORT);
                toast.show();
            }
        });

        Button buttonTF2Default = (Button) findViewById(R.id.buttonTF2Default);
        buttonTF2Default.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                editTextPTX.setText("0.0");
                editTextPTY.setText("0.0");
                editTextPTZ.setText("0.0");
                editTextPRX.setText("0.5");
                editTextPRY.setText("0.5");
                editTextPRZ.setText("0.5");
                editTextPRW.setText("0.5");

                Toast toast = Toast.makeText(v.getContext(), "Transformations for pcl are reset", Toast.LENGTH_SHORT);
                toast.show();
            }
        });

        //----------------------------------------------------------------------------------------//

        // Setting the activity manager for memory info.
        ActivityManager mActivityManager = (ActivityManager) getSystemService(Context.ACTIVITY_SERVICE);
        // Setting Runtime for more memory info.
        Runtime mRuntime = Runtime.getRuntime();

        // Get max memory available in MB.
        totalMem   = mActivityManager.getMemoryClass();
        totalMemLH = mActivityManager.getLargeMemoryClass();
        totalMemMX = mRuntime.maxMemory()/1024/1024;

        //Shared preferences
        prefs = getPreferences(Context.MODE_PRIVATE);

        //        -  -  -              Load preferences              -  -  -                      //
        boolean usePreferences = true;
        //noinspection ConstantConditions
        if (usePreferences) {
            mTangoPoseSensor.loadPreferences();
            mPointCloudSensor.loadPreferences();
            mColorCamera.loadPreferences();
            mFisheyeCamera.loadPreferences();
            mTransformBroadcaster.loadPreferences();

            tf2PTX = prefs.getFloat("/tango_pose/translation/x", tf2PTX);
            tf2PTY = prefs.getFloat("/tango_pose/translation/y", tf2PTY);
            tf2PTZ = prefs.getFloat("/tango_pose/translation/z", tf2PTZ);
            tf2PRX = prefs.getFloat("/tango_pose/rotation/x", tf2PRX);
            tf2PRY = prefs.getFloat("/tango_pose/rotation/y", tf2PRY);
            tf2PRZ = prefs.getFloat("/tango_pose/rotation/z", tf2PRZ);
            tf2PRW = prefs.getFloat("/tango_pose/rotation/w", tf2PRW);
            editTextPTX.setText(String.format(Locale.getDefault(),"%.3f", tf2PTX));
            editTextPTY.setText(String.format(Locale.getDefault(),"%.3f", tf2PTY));
            editTextPTZ.setText(String.format(Locale.getDefault(),"%.3f", tf2PTZ));
            editTextPRX.setText(String.format(Locale.getDefault(),"%.3f", tf2PRX));
            editTextPRY.setText(String.format(Locale.getDefault(),"%.3f", tf2PRY));
            editTextPRZ.setText(String.format(Locale.getDefault(),"%.3f", tf2PRZ));
            editTextPRW.setText(String.format(Locale.getDefault(),"%.3f", tf2PRW));

            checkBoxKeepAwake.setChecked(prefs.getBoolean("/keep_awake", false));

            mWorldTF         = prefs.getString("/tf/world"         , mWorldTF);
            mOdomTF          = prefs.getString("/tf/odom"          , mOdomTF);
            mBaseLinkTF      = prefs.getString("/tf/base_link"     , mBaseLinkTF);
            mColorCameraTF   = prefs.getString("/tf/color_camera"  , mFisheyeCameraTF);
            mFisheyeCameraTF = prefs.getString("/tf/fisheye_camera", mFisheyeCameraTF);

            editTextWorldTF.setText(mWorldTF);
            editTextOdomTF.setText(mOdomTF);
            editTextBaseLinkTF.setText(mBaseLinkTF);
            editTextColorCameraTF.setText(mColorCameraTF);
            editTextFisheyeCameraTF.setText(mFisheyeCameraTF);

        }

        //Launch the timer.
        Handler mHandler = new Handler();
        TimerRunnable timerRunnable = new TimerRunnable();
        mHandler.postDelayed(timerRunnable, updateFrequency);

        Handler mHandler1s = new Handler();
        TimerRunnable1s timerRunnable1s = new TimerRunnable1s();
        mHandler1s.postDelayed(timerRunnable1s, 1000);

        Log.d("TANGO_SENSORS:", "onCreate finished.");
    }

    //-------------------------------UI Helper Functions------------------------------------------//

    //--------------------------------------------------------------------------------------------//
    //--------------------------------------------------------------------------------------------//
    //Update sensor readings on screen. This is usually done with less frequency than the sensors are
    //actually updated, to save resource consumption.
    private class TimerRunnable implements Runnable {
        private Handler mHandler;

        @SuppressLint("SetTextI18n")
        @Override
        public void run() {
            mTangoPoseSensor.updateReadings();
            mPointCloudSensor.updateReadings();

            if (updateEditsMutex) {
                updateEditsMutex = false;

                mTangoPoseSensor.updateEdits();
                mPointCloudSensor.updateEdits();
                mColorCamera.updateEdits();
                mColorCamera.updateReadings();
                mFisheyeCamera.updateEdits();
                mFisheyeCamera.updateReadings();
            }

            // ROS Time
            if (mNode != null) textViewRosTime.setText("Time: "+mNode.getCurrentTime());

            mHandler = new Handler();
            mHandler.postDelayed(this, updateFrequency);

        }
    }

    private class TimerRunnable1s implements Runnable {
        private Handler mHandler;

        @Override
        public void run() {

            // Update FPS information. Possibly needs to be in a separate timer.

            mColorCamera.updateFPS();
            mFisheyeCamera.updateFPS();

            // Update memory information.

            Debug.MemoryInfo memoryInfo = new Debug.MemoryInfo();
            Debug.getMemoryInfo(memoryInfo);

            int totalMemPSS = memoryInfo.getTotalPss()/1024;
            int totalMemPD  = memoryInfo.getTotalPrivateDirty()/1024;
            int totalMemSD  = memoryInfo.getTotalSharedDirty()/1024;

            textViewMemory.setText("Memory: "+String.valueOf(totalMemMX)+"/"+String.valueOf(totalMem)+"("+String.valueOf(totalMemLH)+")"+"\n"+
                "PSS: "+String.valueOf(totalMemPSS)+"| PD: "+String.valueOf(totalMemPD)+"| SD: "+String.valueOf(totalMemSD));

            // Update ROS parameters.
            mTangoPoseSensor.updateParams();
            mPointCloudSensor.updateParams();
            mColorCamera.updateParams();
            mFisheyeCamera.updateParams();

            mHandler = new Handler();
            mHandler.postDelayed(this, 1000);
        }

    }

    //Constructor
    public TangoSensors() {
        super("TangoSensors", "TangoSensors");
    }

    private void initRoutine(NodeMainExecutor nodeMainExecutor){
        // ROS node initialization.
        try {
            runOnUiThread(new Runnable() {
                @SuppressLint("SetTextI18n")
                @Override
                public void run() {
                    TextView textViewMasterURI = (TextView) findViewById(R.id.textViewROSMaster);
                    textViewMasterURI.setText("ROS Master: " + getMasterUri());
                }
            });

            java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
            java.net.InetAddress local_network_address = socket.getLocalAddress();

            if (getMasterUri().toString().equals("")) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        ImageView imageViewMasterStatus = (ImageView) findViewById(R.id.imageViewMasterStatus);
                        imageViewMasterStatus.setImageResource(R.drawable.circle_yellow);
                    }
                });
            }

            NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
            nodeMainExecutor.execute(this, nodeConfiguration);
        } catch (IOException e) {
            Log.i(TAG,"Exception", e);
        }
    }

    //------------------------------ ROS NODE INITIALIZATION -------------------------------------//
    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        // Finally permit Tango start, it should not start in the ROS MASTER selection.
        tangoStartPermission = true;
        stopTango();
        startTango();

        initRoutine(nodeMainExecutor);
    }

    @Override
    public GraphName getDefaultNodeName() {
        String mNodeName = "tango_sensors_node";
        return GraphName.of(mNodeName);
    }

    private void registerTopics(final ConnectedNode node) {
        // Setting Parameters: ROS Parameters have higher priority, if they are not available - preferences from the last usage will be used.
        // If no ROS parameters and preferences are available, default values are used.
        ParameterTree params = node.getParameterTree();
        boolean useServerParams = true;
        //noinspection ConstantConditions
        if (useServerParams) {
            mTangoPoseSensor.sleep = params.getInteger(GraphName.of("/tango_sensors/tango_pose/sleep"), mTangoPoseSensor.sleep);
            mTangoPoseSensor.enabled = params.getBoolean(GraphName.of("/tango_sensors/tango_pose/enabled"), mTangoPoseSensor.enabled);

            mPointCloudSensor.sleep = params.getInteger(GraphName.of("/tango_sensors/point_cloud/sleep"), mPointCloudSensor.sleep);
            mPointCloudSensor.enabled = params.getBoolean(GraphName.of("/tango_sensors/point_cloud/enabled"), mPointCloudSensor.enabled);
        }

        //---------------------------------------TRANSFORM----------------------------------------//
        mTransformBroadcaster.setNode(node);
        mTransformBroadcaster.prepareNode();
        mTransformBroadcaster.executeNode();

        //----------------------------------------TANGO: POSE-------------------------------------//
        mTangoPoseSensor.setNode(node);
        mTangoPoseSensor.prepareNode();
        mTangoPoseSensor.executeNode();

        //----------------------------------------TANGO: POINT CLOUD-------------------------------------//
        mPointCloudSensor.setNode(node);
        mPointCloudSensor.prepareNode();
        mPointCloudSensor.executeNode();

        // Color camera
        mColorCamera.setNode(node);
        mColorCamera.loadServerParams();
        mColorCamera.prepareNode();
        mColorCamera.executeNode();

        // Fisheye Camera
        mFisheyeCamera.setNode(node);
        mFisheyeCamera.loadServerParams();
        mFisheyeCamera.prepareNode();
        mFisheyeCamera.executeNode();

        //Update view with received parameters
        updateEditsMutex = true;

    }

    //
    private void shutdownTopics() {
        mTangoPoseSensor.shutdownNode();
        mPointCloudSensor.shutdownNode();
        mColorCamera.shutdownNode();
        mFisheyeCamera.shutdownNode();
        mTransformBroadcaster.shutdownNode();
    }

    //--------------------------THE "tango_sensors" ROS NODE IMPLEMENTATION-----------------------**
    @Override
    public void onStart(final ConnectedNode node) {
        Log.i("tango_sensors", "Node Started");
        mNode = node;
        //Registering Topics
        registerTopics(node);

        //Registering Pose Reset subscriber
        Subscriber<Empty> mResetPoseSubscriber = node.newSubscriber("/tango_sensors/reset_pose", Empty._TYPE);
        mResetPoseSubscriber.addMessageListener(new MessageListener<Empty>() {
            @Override
            public void onNewMessage(Empty empty) {
                Log.i("TANGO POSE RESET!","TANGO POSE RESET!");
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        resetTango();
                    }
                });
            }
        });
    }

    @Override
    public void onShutdown(Node node) {
        Log.i("ts/accelerometer", "Node Shutdown");

        shutdownTopics();
    }

    @Override
    public void onShutdownComplete(Node node) {
        Log.i("ts/accelerometer", "Node Shutdown Complete");
    }

    @Override
    public void onError(Node node, Throwable throwable) {
        Log.e("ts/accelerometer", "Error occurred");

    }

    //------------------------------TANGO RELATED STUFF-------------------------------------------//
    private TangoConfig setupTangoConfig(Tango tango) {
        // Create a new Tango Configuration and enable the HelloMotionTrackingActivity API.
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);

        // Motion tracking, tango_pose, also is responsible for fisheye camera.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, mTangoPoseSensor.enabled || mFisheyeCamera.enabled);

        // Tango service should automatically attempt to recover when it enters an invalid state.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);

        // Allow the usage of depth sensor.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, mPointCloudSensor.enabled);

        // Enable point cloud mode.
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);

        // Allow usage of color camera.
        config.putBoolean(TangoConfig.KEY_BOOLEAN_COLORCAMERA, mColorCamera.enabled);

        return config;
    }

    // Log TANGO:Pose data, apply the transformation quaternion;
    private void logTangoPose(TangoPoseData pose) {
        float translation[] = pose.getTranslationAsFloats();
        float orientation[] = pose.getRotationAsFloats();

        float dPoseTX = translation[1];
        float dPoseTY = -translation[0];
        float dPoseTZ = translation[2];

        float dPoseOX = orientation[2];
        float dPoseOY = -orientation[0];
        float dPoseOZ = orientation[1];
        float dPoseOW = -orientation[3];

        float resw = innerPoseOW * dPoseOW - innerPoseOX * dPoseOX - innerPoseOY * dPoseOY - innerPoseOZ * dPoseOZ;
        float resx = innerPoseOW * dPoseOX + innerPoseOX * dPoseOW + innerPoseOY * dPoseOZ - innerPoseOZ * dPoseOY;
        float resy = innerPoseOW * dPoseOY - innerPoseOX * dPoseOZ + innerPoseOY * dPoseOW + innerPoseOZ * dPoseOX;
        float resz = innerPoseOW * dPoseOZ + innerPoseOX * dPoseOY - innerPoseOY * dPoseOX + innerPoseOZ * dPoseOW;

        mTangoPoseSensor.setValue(dPoseTX, dPoseTY, dPoseTZ, resx, resy, resz, resw);
        mTangoPoseSensor.sendData();
    }

    //--------------------------------------------------------------------------------------------//
    void startTango() {
        //Resume TANGO sensors.
        Log.d("TANGO ROUTINE:", "startTango()");

        if (!tangoStartPermission) {
            Log.i(TAG, "Tango start is not permitted yet.");

            return;
        }

        if (tangoStarted) {
            Log.i(TAG, "Tango already started.");

            return;
        }

        tangoStarted = false;
        // Start TANGO functions.
        mTango = new Tango(this, new Runnable() {
            @Override
            public void run() {
                synchronized (this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);

                        synchronized (this) {
                            for (Map.Entry<Integer, Integer> entry : cameraTextures_.entrySet())
                                mTango.connectTextureId(entry.getKey(), entry.getValue());
                        }

                        final ArrayList<TangoCoordinateFramePair> framePairs =
                                new ArrayList<TangoCoordinateFramePair>();
                        framePairs.add(new TangoCoordinateFramePair(
                                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                TangoPoseData.COORDINATE_FRAME_DEVICE));

                        // Listen for new Tango data
                        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
                            @Override
                            public void onPoseAvailable(final TangoPoseData pose) {
                                if (!mTangoPoseSensor.enabled) {
                                    return;
                                }
                                logTangoPose(pose);
                            }

                            @SuppressWarnings("PointlessArithmeticExpression")
                            @Override
                            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {
                                if (!mPointCloudSensor.enabled) {
                                    return;
                                }
                                // Getting the Range
                                double rng = 0.0;
                                for (int i=0;i<pointCloud.numPoints; i++){
                                    double pt_x = pointCloud.points.get(i*4 + 0);
                                    double pt_y = pointCloud.points.get(i*4 + 1);
                                    double pt_z = pointCloud.points.get(i*4 + 2);

                                    if ((pt_x > -0.10) && (pt_x < 0.10) && (pt_y > -0.25) && (pt_y < -0.10)){
                                        rng = rng + pt_z;
                                    }
                                }

                                while (mPointCloudSensor.sending.get()) {
                                    sleep(1);
                                }
                                synchronized (this) {
                                    mPointCloudSensor.setValue(pointCloud,
                                            mTangoPoseSensor.tx, mTangoPoseSensor.ty, mTangoPoseSensor.tz,
                                            mTangoPoseSensor.ox, mTangoPoseSensor.oy, mTangoPoseSensor.oz, mTangoPoseSensor.ow);
                                    }

                                mPointCloudSensor.changed.set(true);
                                mPointCloudSensor.sendData();
                            }

                            @Override
                            public void onFrameAvailable(int cameraId) {

                                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                                    if (mColorCamera.enabled) {
                                        mColorCamera.view.requestRender();
                                        mColorCamera.cameraFrameCounter++;
                                        mColorCamera.sendData();
                                    }
                                } else if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE) {
                                    if (mFisheyeCamera.enabled) {
                                        mFisheyeCamera.view.requestRender();
                                        mFisheyeCamera.cameraFrameCounter++;
                                        mFisheyeCamera.sendData();
                                    }
                                }
                            }
                        });
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, "OutOfDate Exception");
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "Tango Error");
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, "Tango Invalid Exception");
                    } finally {
                        Log.i(TAG, "TANGO STARTED!");
                        tangoStarted = true;
                    }
                }
            }
        });

    }

    //--------------------------------------------------------------------------------------------//
    void stopTango() {
        //Close TANGO sensors.
        Log.d("TANGO ROUTINE:", "stopTango()");

        if (mTango == null) {
            tangoStarted = false;
            return;
        }

        synchronized (this) {
            try {
                mTango.disconnect();
            } catch (TangoErrorException e) {
                Log.e("Tango Error", "Tango Error Exception");
            }
        }

        tangoStarted = false;
    }

    //--------------------------------------------------------------------------------------------//
    private void resetTango() {
        synchronized (this) {
            try {
                mTango.resetMotionTracking();
            } catch (Exception e) {
                Toast toast = Toast.makeText(this, "Error during the motion tracking reset", Toast.LENGTH_SHORT);
                toast.show();
            } finally {
                Toast toast = Toast.makeText(this, "Tango system has been reset", Toast.LENGTH_SHORT);
                toast.show();
            }
        }
    }

    //---------------------------------------onResume---------------------------------------------//
    @Override
    protected void onResume() {
        Log.d("TANGO_SENSORS:", "onResume();");

        super.onResume();
        //startTango();

        Log.d("TANGO_SENSORS:", "onResume() finished;");
    }

    //-------------------------------------onPause------------------------------------------------//
    @Override
    protected void onPause() {
        Log.d("TANGO_SENSORS:", "onPause();");

        super.onPause();
        //stopTango();

        Log.d("TANGO_SENSORS:", "onPause() finished;");
    }

    @SuppressWarnings("UnusedParameters")
    public void onButtonExitClick(View view) {
        shutdownTopics();
        mNode.shutdown();
        stopTango();
        finish();
    }

    @SuppressWarnings("UnusedParameters")
    public void onButtonResetTangoClick(View view) {
        resetTango();
    }

    //------------------------------------- OPENGL -----------------------------------------------//
    public synchronized void attachTexture(final int cameraId, final int textureName) {
        if (textureName > 0) {

            // Link the texture with Tango if the texture changes after
            // Tango is connected. This generally doesn't happen but
            // technically could because they happen in separate
            // threads. Otherwise the link will be made in startTango().
            if (tangoStarted && cameraTextures_.get(cameraId) != textureName)
                mTango.connectTextureId(cameraId, textureName);
            cameraTextures_.put(cameraId, textureName);
        } else
            cameraTextures_.remove(cameraId);
    }

    public Point getCameraFrameSize(int cameraId) {
        if ((mTango == null) || (!tangoStarted)) {
            if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                //return new Point(1024, 768);
                return new Point(1280,720);
                //return new Point(1024/4,768/4);
            }
            if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_FISHEYE) {
                //return new Point(640,480);
                return new Point(640, 480);
            }
        } else {
            TangoCameraIntrinsics intrinsics = mTango.getCameraIntrinsics(cameraId);
            return new Point(intrinsics.width, intrinsics.height);
        }
        return new Point(0, 0);
    }

    public synchronized void updateTexture(int cameraId) {
        if (tangoStarted) {
            try {
                mTango.updateTexture(cameraId);
            } catch (TangoInvalidException e) {
                e.printStackTrace();
            }
        }
    }
}
