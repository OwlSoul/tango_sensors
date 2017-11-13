package edu.nctu.arlab.tango_sensors;

import android.annotation.SuppressLint;
import android.content.SharedPreferences;
import android.os.Handler;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.ToggleButton;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

import geometry_msgs.PoseStamped;
import geometry_msgs.TransformStamped;
import std_msgs.Header;

import static android.content.ContentValues.TAG;

class SensorTangoPose {
    // Readings
    float   tx        = 0.00f;
    float   ty        = 0.00f;
    float   tz        = 0.00f;
    float   ox        = 0.00f;
    float   oy        = 0.00f;
    float   oz        = 0.00f;
    float   ow        = 1.00f;
    // Parameters
    boolean enabled  = false;
    int     sleep    = 50;
    private static final int minSleep = 2;      //Minimal allowed sleep

    // ROS updates
    private boolean uEnabled = false;
    private boolean uSleep   = false;
    //Flags
    private final AtomicBoolean changed     = new AtomicBoolean(false);
    private final AtomicBoolean canSend     = new AtomicBoolean(true);
    private boolean nodeStarted      = false;
    //Names
    private final String mTopicName = "tango_pose";
    //ROS classes and variables
    private int               sequenceNumber = 0;
    private ConnectedNode              node = null;

    private Publisher<PoseStamped>        mPosePublisher;              //High frequency TANGO: Pose

    private tf2_msgs.TFMessage             mTFTangoPose;
    private geometry_msgs.TransformStamped mTFTangoPoseTransform;
    private geometry_msgs.Transform        mTFTangoPoseTransformData;
    private std_msgs.Header                mTFTangoPoseHeader;
    private geometry_msgs.Vector3          mTFTangoPoseTranslate;
    private geometry_msgs.Quaternion       mTFTangoPoseRotate;

    private geometry_msgs.PoseStamped      mPose;
    private std_msgs.Header                mPoseHeader;
    private geometry_msgs.Pose             mPoseData;
    private geometry_msgs.Point            mPoseTranslation;
    private geometry_msgs.Quaternion       mPoseOrientation;

    //Views
    private final TextView     textViewTX;
    private final TextView     textViewTY;
    private final TextView     textViewTZ;
    private final TextView     textViewOX;
    private final TextView     textViewOY;
    private final TextView     textViewOZ;
    private final TextView     textViewOW;
    private final EditText     editTextSleep;
    private final ImageView    imageViewIndicator;
    private final ToggleButton toggleButtonEnabled;

    // TangoSensors main class
    final private TangoSensors mTangoSensors;
    final private TransformBroadcaster mTransformBroadcaster;

    private class DataSender2 implements Runnable {
        @Override
        public void run() {
            if (!nodeStarted) return;

            try {
                //Pose Transform publishing.
                if ( (mTransformBroadcaster.mTransformPublisher != null) ) {
                    // TF2: "/tango_pose" related to "/tango_origin".
                    mTFTangoPoseHeader.setSeq(0);
                    mTFTangoPoseHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mOdomTF);
                    mTFTangoPoseHeader.setStamp(node.getCurrentTime());
                    mTFTangoPoseTransform.setHeader(mTFTangoPoseHeader);
                    mTFTangoPoseTransform.setChildFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mBaseLinkTF);
                    mTFTangoPoseTranslate.setX(tx);
                    mTFTangoPoseTranslate.setY(ty);
                    mTFTangoPoseTranslate.setZ(tz);
                    mTFTangoPoseRotate.setX(ox);
                    mTFTangoPoseRotate.setY(oy);
                    mTFTangoPoseRotate.setZ(oz);
                    mTFTangoPoseRotate.setW(ow);
                    mTFTangoPoseTransformData.setTranslation(mTFTangoPoseTranslate);
                    mTFTangoPoseTransformData.setRotation(mTFTangoPoseRotate);
                    mTFTangoPoseTransform.setTransform(mTFTangoPoseTransformData);

                    List<TransformStamped> mTFTangoPoseList = new ArrayList<TransformStamped>();
                    mTFTangoPoseList.add(mTFTangoPoseTransform);
                    mTFTangoPose.setTransforms(mTFTangoPoseList);

                    mTransformBroadcaster.mTransformPublisher.publish(mTFTangoPose);
                }

                //The actual Pose message
                if ( (mPosePublisher != null) ) {
                    mPoseHeader.setStamp(node.getCurrentTime());
                    mPoseHeader.setSeq(++sequenceNumber);
                    mPoseHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mOdomTF);
                    mPoseTranslation.setX(tx);
                    mPoseTranslation.setY(ty);
                    mPoseTranslation.setZ(tz);
                    mPoseOrientation.setX(ox);
                    mPoseOrientation.setY(oy);
                    mPoseOrientation.setZ(oz);
                    mPoseOrientation.setW(ow);
                    mPoseData.setPosition(mPoseTranslation);
                    mPoseData.setOrientation(mPoseOrientation);
                    mPose.setHeader(mPoseHeader);
                    mPose.setPose(mPoseData);
                    mPosePublisher.publish(mPose);
                }

            } catch (Exception e) {
                Log.e(TAG, "exception", e);
            }

            changed.set(false);
        }
    }

    //------------------------------------- CONSTRUCTOR ------------------------------------------//
    SensorTangoPose(TangoSensors pTangoSensors, TransformBroadcaster pTransformBroadcaster) {
        mTangoSensors = pTangoSensors;
        mTransformBroadcaster = pTransformBroadcaster;
        //------------------------------------Changeable----------------------------------------//

        textViewTX           = (TextView)     mTangoSensors.findViewById(R.id.textViewPoseTX);
        textViewTY           = (TextView)     mTangoSensors.findViewById(R.id.textViewPoseTY);
        textViewTZ           = (TextView)     mTangoSensors.findViewById(R.id.textViewPoseTZ);
        textViewOX           = (TextView)     mTangoSensors.findViewById(R.id.textViewPoseOX);
        textViewOY           = (TextView)     mTangoSensors.findViewById(R.id.textViewPoseOY);
        textViewOZ           = (TextView)     mTangoSensors.findViewById(R.id.textViewPoseOZ);
        textViewOW           = (TextView)     mTangoSensors.findViewById(R.id.textViewPoseCos);
        editTextSleep        = (EditText)     mTangoSensors.findViewById(R.id.editTextPose);

        imageViewIndicator  = (ImageView)    mTangoSensors.findViewById(R.id.imageViewPose);
        toggleButtonEnabled = (ToggleButton) mTangoSensors.findViewById(R.id.toggleButtonPose);

        editTextSleep.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @SuppressWarnings("ConstantConditions")
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

        toggleButtonEnabled.setChecked(enabled);
        toggleButtonEnabled.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @SuppressWarnings("ConstantConditions")
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    imageViewIndicator.setImageResource(R.drawable.circle_green);
                    startSensor();
                    prepareNode();
                    executeNode();
                } else {
                    imageViewIndicator.setImageResource(R.drawable.circle_red);
                    stopSensor();
                    prepareNode();
                    executeNode();
                }
                enabled = toggleButtonEnabled.isChecked();

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
    }

    //------------------------------------ Start Sensor ------------------------------------------//
    private void startSensor() {
        Log.i("Tango Pose Sensor", "startSensor()");

        if (mTangoSensors.tangoStarted) mTangoSensors.stopTango();
        mTangoSensors.startTango();
        enabled = true;
    }

    //------------------------------------ Stop Sensor -------------------------------------------//
    private void stopSensor() {
        Log.i("Tango Pose Sensor", "stopSensor()");

        if (mTangoSensors.tangoStarted) mTangoSensors.stopTango();

        tx = 0.00f;
        ty = 0.00f;
        tz = 0.00f;
        ox = 0.00f;
        oy = 0.00f;
        oz = 0.00f;
        ow = 1.00f;
        updateReadings();

        enabled = false;
        mTangoSensors.startTango(); //Tango should be restarted since other sensors still might use it.
    }

    //----------------------------------- Update Readings ----------------------------------------//
    @SuppressLint("SetTextI18n")
    void updateReadings(){
        if (enabled) {
            textViewTX.setText("X: " + String.format(Locale.getDefault(), "%.4f", tx));
            textViewTY.setText("Y: " + String.format(Locale.getDefault(), "%.4f", ty));
            textViewTZ.setText("Z: " + String.format(Locale.getDefault(), "%.4f", tz));

            textViewOX.setText("X: " + String.format(Locale.getDefault(), "%.4f", ox));
            textViewOY.setText("Y: " + String.format(Locale.getDefault(), "%.4f", oy));
            textViewOZ.setText("Z: " + String.format(Locale.getDefault(), "%.4f", oz));
            textViewOW.setText("Z: " + String.format(Locale.getDefault(), "%.4f", ow));
        }
    }

    //------------------------------------ Update Edits ------------------------------------------//
    void updateEdits(){
        editTextSleep.setText(String.valueOf(sleep));
        toggleButtonEnabled.setChecked(enabled);
    }

    //---------------------------------- Load Preferences ----------------------------------------//
    void loadPreferences(){
        sleep   = mTangoSensors.prefs.getInt("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/"+"sleep", sleep);
        enabled = mTangoSensors.prefs.getBoolean("/"+ mTangoSensors.mNodeNamespace +"/"+mTopicName+"/"+"enabled", enabled);
        updateEdits();
    }

    //----------------------------------- Set Value ----------------------------------------------//
    void setValue(float tx, float ty, float tz, float ox, float oy, float oz, float ow){
        this.tx = tx;
        this.ty = ty;
        this.tz = tz;
        this.ox = ox;
        this.oy = oy;
        this.oz = oz;
        this.ow = ow;
        changed.set(true);
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

        mPosePublisher = node.newPublisher("tango_sensors/tango_pose", "geometry_msgs/PoseStamped");

        mTFTangoPose              = node.getTopicMessageFactory().newFromType(tf2_msgs.TFMessage._TYPE);
        mTFTangoPoseTransform     = node.getTopicMessageFactory().newFromType(geometry_msgs.TransformStamped._TYPE);
        mTFTangoPoseTransformData = node.getTopicMessageFactory().newFromType(geometry_msgs.Transform._TYPE);
        mTFTangoPoseHeader        = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        mTFTangoPoseTranslate     = node.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
        mTFTangoPoseRotate        = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);

        mPose                     = node.getTopicMessageFactory().newFromType(geometry_msgs.PoseStamped._TYPE);
        mPoseHeader               = node.getTopicMessageFactory().newFromType(Header._TYPE);
        mPoseData                 = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
        mPoseTranslation          = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
        mPoseOrientation          = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);

        sequenceNumber = 0;
    }

    //--------------------------------- ROS: Shutdown Node ---------------------------------------//
    void shutdownNode() {
        nodeStarted = false;

        sequenceNumber = 0;
        if (mPosePublisher == null) return;
        mPosePublisher.shutdown();
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

        if (uSleep) {
            uSleep = false;
            ParameterTree params = node.getParameterTree();
            params.set(GraphName.of("/tango_sensors/tango_pose/sleep"), sleep);
        }
        if (uEnabled) {
            uEnabled = false;
            ParameterTree params = node.getParameterTree();
            params.set(GraphName.of("/tango_sensors/tango_pose/enabled"), enabled);
        }
    }
}
