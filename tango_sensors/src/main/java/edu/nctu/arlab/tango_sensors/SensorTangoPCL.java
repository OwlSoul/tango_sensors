package edu.nctu.arlab.tango_sensors;

import android.annotation.SuppressLint;
import android.content.SharedPreferences;
import android.os.AsyncTask;
import android.os.Handler;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.ToggleButton;

import com.google.atap.tangoservice.TangoPointCloudData;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import sensor_msgs.PointField;
import std_msgs.Header;
import std_msgs.Time;

import static android.content.ContentValues.TAG;

class SensorTangoPCL {
    //Readings
    private float   tx        = 0.00f; //Pose when PCL was taken.
    private float   ty        = 0.00f;
    private float   tz        = 0.00f;
    private float   ox        = 0.00f;
    private float   oy        = 0.00f;
    private float   oz        = 0.00f;
    private float   ow        = 1.00f;

    private int     points    = 0;
    private static  TangoPointCloudData mPointCloudData = null;

    //Parameters
    boolean enabled  = false;
    int     sleep    = 0;
    private final static int minSleep = 100;

    //ROS updates
    private boolean uEnabled = false;
    private boolean uSleep   = false;
    //Flags
    final AtomicBoolean changed            = new AtomicBoolean(false);
    final AtomicBoolean sending            = new AtomicBoolean(false);
    private final AtomicBoolean canSend    = new AtomicBoolean(true);
    private boolean nodeStarted      = false;
    //Names
    private final String mTopicName    = "point_cloud";
    //ROS classes and variables
    private int               sequenceNumber = 0;
    private ConnectedNode     node = null;

    private Publisher<geometry_msgs.PoseStamped> mPosePCLPublisher;      //Low frequency pose, sent when point cloud was taken.
    private Publisher<sensor_msgs.PointCloud2>   mPointCloudPublisher;

    private tf2_msgs.TFMessage             mTFTangoPosePCL;
    private geometry_msgs.TransformStamped mTFTangoPosePCLTransform;
    private geometry_msgs.Transform        mTFTangoPosePCLTransformData;
    private std_msgs.Header                mTFTangoPosePCLHeader;
    private geometry_msgs.Vector3          mTFTangoPosePCLTranslate;
    private geometry_msgs.Quaternion       mTFTangoPosePCLRotate;

    private tf2_msgs.TFMessage             mTFPointCloudOrigin;
    private geometry_msgs.TransformStamped mTFPointCloudTransform;
    private geometry_msgs.Transform        mTFPointCloudTransformData;
    private std_msgs.Header                mTFPointCloudHeader;
    private geometry_msgs.Vector3          mTFPointCloudTranslate;
    private geometry_msgs.Quaternion       mTFPointCloudRotate;

    private geometry_msgs.PoseStamped      mPosePCL;
    private std_msgs.Header                mPosePCLHeader;
    private geometry_msgs.Pose             mPosePCLData;
    private geometry_msgs.Point            mPosePCLTranslation;
    private geometry_msgs.Quaternion       mPosePCLOrientation;

    private sensor_msgs.PointCloud2        mPointCloud;
    private int                            PointStep = 4 * 4; // One field is 4 bytes (FLOAT32), we have 4 fields.
    private std_msgs.Header                mPointCloudHeader;

    //Views
    private final TextView     textViewPoints;
    private final EditText     editTextSleep;
    private final ImageView    imageViewIndicator;
    private final ToggleButton toggleButtonEnabled;

    //TangoSensors main class
    private final TangoSensors mTangoSensors;
    private final TransformBroadcaster mTransformBroadcaster;

    private class DataSender extends AsyncTask<Void, Void, Void> {
        @SuppressWarnings("PointlessArithmeticExpression")
        @Override
        protected Void doInBackground(Void... params) {
            int bits;
            float ptX, ptY, ptZ, ptC;

            if (!nodeStarted) return null;

            try {
                if ((mPointCloudPublisher != null) && (mPointCloudData != null) &&
                        (!sending.get()) &&
                        (((sleep > 0) && (canSend.get())) || ((sleep == 0)))) {
                    sending.set(true);
                    canSend.set(false);

                    org.ros.message.Time t = node.getCurrentTime();

                    // TF2: "/tango_pose_pcl" related to "/tango_origin".
                    mTFTangoPosePCLHeader.setSeq(0);
                    mTFTangoPosePCLHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mOdomTF);
                    mTFTangoPosePCLHeader.setStamp(t);
                    mTFTangoPosePCLTransform.setHeader(mTFTangoPosePCLHeader);
                    mTFTangoPosePCLTransform.setChildFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mBaseLinkTF+"/"+"pcl");
                    mTFTangoPosePCLTranslate.setX(tx);
                    mTFTangoPosePCLTranslate.setY(ty);
                    mTFTangoPosePCLTranslate.setZ(tz);
                    mTFTangoPosePCLRotate.setX(ox);
                    mTFTangoPosePCLRotate.setY(oy);
                    mTFTangoPosePCLRotate.setZ(oz);
                    mTFTangoPosePCLRotate.setW(ow);
                    mTFTangoPosePCLTransformData.setTranslation(mTFTangoPosePCLTranslate);
                    mTFTangoPosePCLTransformData.setRotation(mTFTangoPosePCLRotate);
                    mTFTangoPosePCLTransform.setTransform(mTFTangoPosePCLTransformData);

                    List<geometry_msgs.TransformStamped> mTangoOriginList = new ArrayList<geometry_msgs.TransformStamped>();
                    mTangoOriginList.add(mTFTangoPosePCLTransform);
                    mTFTangoPosePCL.setTransforms(mTangoOriginList);

                    mTransformBroadcaster.mTransformPublisher.publish(mTFTangoPosePCL);

                    // TF2: "/tango_point_cloud" related to "/tango_pose_pcl"
                    mTFPointCloudHeader.setSeq(0);
                    mTFPointCloudHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mBaseLinkTF+"/"+"pcl");
                    mTFPointCloudHeader.setStamp(t);
                    mTFPointCloudTransform.setHeader(mTFPointCloudHeader);
                    mTFPointCloudTransform.setChildFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mBaseLinkTF+"/"+"point_cloud");
                    mTFPointCloudTranslate.setX(mTangoSensors.tf2PTX);
                    mTFPointCloudTranslate.setY(mTangoSensors.tf2PTY);
                    mTFPointCloudTranslate.setZ(mTangoSensors.tf2PTZ);
                    mTFPointCloudRotate.setX(mTangoSensors.tf2PRX);
                    mTFPointCloudRotate.setY(mTangoSensors.tf2PRY);
                    mTFPointCloudRotate.setZ(mTangoSensors.tf2PRZ);
                    mTFPointCloudRotate.setW(mTangoSensors.tf2PRW);
                    mTFPointCloudTransformData.setTranslation(mTFPointCloudTranslate);
                    mTFPointCloudTransformData.setRotation(mTFPointCloudRotate);
                    mTFPointCloudTransform.setTransform(mTFPointCloudTransformData);

                    List<geometry_msgs.TransformStamped> mTFPose2List = new ArrayList<geometry_msgs.TransformStamped>();
                    mTFPose2List.add(mTFPointCloudTransform);
                    mTFPointCloudOrigin.setTransforms(mTFPose2List);

                    mTransformBroadcaster.mTransformPublisher.publish(mTFPointCloudOrigin);

                    //Pose when point cloud was taken.
                    mPosePCLHeader.setStamp(t);
                    mPosePCLHeader.setSeq(++sequenceNumber);
                    mPosePCLHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mOdomTF);
                    mPosePCLTranslation.setX(tx);
                    mPosePCLTranslation.setY(ty);
                    mPosePCLTranslation.setZ(tz);
                    mPosePCLOrientation.setX(ox);
                    mPosePCLOrientation.setY(oy);
                    mPosePCLOrientation.setZ(oz);
                    mPosePCLOrientation.setW(ow);
                    mPosePCLData.setPosition(mPosePCLTranslation);
                    mPosePCLData.setOrientation(mPosePCLOrientation);
                    mPosePCL.setHeader(mPosePCLHeader);
                    mPosePCL.setPose(mPosePCLData);
                    mPosePCLPublisher.publish(mPosePCL);

                    // Compose the Point Cloud message. It's a big one.
                    mPointCloudHeader.setStamp(t);
                    mPointCloudHeader.setSeq(++sequenceNumber);
                    mPointCloudHeader.setFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mBaseLinkTF+"/"+"point_cloud");
                    mPointCloud.setHeader(mPointCloudHeader);
                    mPointCloud.setHeight(1);
                    synchronized (this) {
                        mPointCloud.setWidth(mPointCloudData.numPoints);

                        byte[] data = new byte[mPointCloudData.numPoints * PointStep];
                        for (int i = 0; i < mPointCloudData.numPoints; i++) {
                            ptX = mPointCloudData.points.get(i * 4 + 0);
                            ptY = mPointCloudData.points.get(i * 4 + 1);
                            ptZ = mPointCloudData.points.get(i * 4 + 2);
                            ptC = mPointCloudData.points.get(i * 4 + 3);

                            bits = Float.floatToIntBits(ptX);
                            data[(i * PointStep) + 0 + 0] = (byte) ((bits) & 0xff);
                            data[(i * PointStep) + 0 + 1] = (byte) ((bits >> 8) & 0xff);
                            data[(i * PointStep) + 0 + 2] = (byte) ((bits >> 16) & 0xff);
                            data[(i * PointStep) + 0 + 3] = (byte) ((bits >> 24) & 0xff);

                            bits = Float.floatToIntBits(ptY);
                            data[(i * PointStep) + 4 + 0] = (byte) ((bits) & 0xff);
                            data[(i * PointStep) + 4 + 1] = (byte) ((bits >> 8) & 0xff);
                            data[(i * PointStep) + 4 + 2] = (byte) ((bits >> 16) & 0xff);
                            data[(i * PointStep) + 4 + 3] = (byte) ((bits >> 24) & 0xff);

                            bits = Float.floatToIntBits(ptZ);
                            data[(i * PointStep) + 8 + 0] = (byte) ((bits) & 0xff);
                            data[(i * PointStep) + 8 + 1] = (byte) ((bits >> 8) & 0xff);
                            data[(i * PointStep) + 8 + 2] = (byte) ((bits >> 16) & 0xff);
                            data[(i * PointStep) + 8 + 3] = (byte) ((bits >> 24) & 0xff);

                            bits = Float.floatToIntBits(ptC);
                            data[(i * PointStep) + 12 + 0] = (byte) ((bits) & 0xff);
                            data[(i * PointStep) + 12 + 1] = (byte) ((bits >> 8) & 0xff);
                            data[(i * PointStep) + 12 + 2] = (byte) ((bits >> 16) & 0xff);
                            data[(i * PointStep) + 12 + 3] = (byte) ((bits >> 24) & 0xff);
                        }

                        ChannelBufferOutputStream stream;
                        stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
                        try {
                            stream.write(data);
                        } catch (Exception e) {
                            Log.e(TAG,"Exception",e);
                        }

                        mPointCloud.setData(stream.buffer().copy());
                        mPointCloud.setIsBigendian(false);
                        mPointCloud.setIsDense(true);
                        mPointCloud.setRowStep(mPointCloudData.numPoints * PointStep);
                    }

                    mPointCloudPublisher.publish(mPointCloud);

                    sending.set(false);
                }

            } catch (Exception e) {
                Log.e(TAG, "exception", e);
                sending.set(false);
            }

            changed.set(false);

            return null;
        }

        @Override
        protected void onPostExecute(Void result) {
            changed.set(false);

            if (sleep > 0) {
                // Setting delay after sleep
                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        int iSleep = (sleep < minSleep) ? minSleep : sleep;
                        try {
                            Thread.sleep(iSleep);
                        } catch (Exception e) {
                            Log.e(TAG, "Exception", e);
                        }
                        canSend.set(true);
                    }
                }).start();
            }
            else {
                canSend.set(true);
            }
        }
    }

    //------------------------------------- CONSTRUCTOR ------------------------------------------//
    SensorTangoPCL(TangoSensors pTangoSensors, TransformBroadcaster pTransformBroadcaster) {
        mTangoSensors = pTangoSensors;
        mTransformBroadcaster = pTransformBroadcaster;
        //------------------------------------Changeable----------------------------------------//

        textViewPoints      = (TextView)     mTangoSensors.findViewById(R.id.textViewPointCloudPoints);
        editTextSleep       = (EditText)     mTangoSensors.findViewById(R.id.editTextPointCloud);
        imageViewIndicator  = (ImageView)    mTangoSensors.findViewById(R.id.imageViewPointCloud);
        toggleButtonEnabled = (ToggleButton) mTangoSensors.findViewById(R.id.toggleButtonPointCloud);

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
                    shutdownNode();
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
        Log.i("Point Cloud Sensor", "startSensor()");

        if (mTangoSensors.tangoStarted) mTangoSensors.stopTango();
        mTangoSensors.startTango();

        enabled = true;
    }

    //------------------------------------ Stop Sensor -------------------------------------------//
    private void stopSensor() {
        Log.i("Point Cloud Sensor", "stopSensor()");

        if (mTangoSensors.tangoStarted) mTangoSensors.stopTango();

        points = 0;
        updateReadings();

        enabled = false;
        mTangoSensors.startTango(); //Tango should be restarted since other sensors still might use it.
    }

    //----------------------------------- Update Readings ----------------------------------------//
    @SuppressLint("SetTextI18n")
    void updateReadings(){
        if (enabled) {
            textViewPoints.setText("Points: " + String.valueOf(points));
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
    void setValue(TangoPointCloudData pTangoPointCloudData, float tx, float ty, float tz,
                         float ox, float oy, float oz, float ow){
        mPointCloudData = pTangoPointCloudData;
        points = mPointCloudData.numPoints;
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

        mPosePCLPublisher = node.newPublisher("tango_sensors/tango_pose_pcl", "geometry_msgs/PoseStamped");
        mPointCloudPublisher = node.newPublisher("tango_sensors/point_cloud", "sensor_msgs/PointCloud2");

        mTFTangoPosePCL = node.getTopicMessageFactory().newFromType(tf2_msgs.TFMessage._TYPE);
        mTFTangoPosePCLTransform = node.getTopicMessageFactory().newFromType(geometry_msgs.TransformStamped._TYPE);
        mTFTangoPosePCLTransformData = node.getTopicMessageFactory().newFromType(geometry_msgs.Transform._TYPE);
        mTFTangoPosePCLHeader = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        mTFTangoPosePCLTranslate = node.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
        mTFTangoPosePCLRotate = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);

        mTFPointCloudOrigin = node.getTopicMessageFactory().newFromType(tf2_msgs.TFMessage._TYPE);
        mTFPointCloudTransform = node.getTopicMessageFactory().newFromType(geometry_msgs.TransformStamped._TYPE);
        mTFPointCloudTransformData = node.getTopicMessageFactory().newFromType(geometry_msgs.Transform._TYPE);
        mTFPointCloudHeader = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        mTFPointCloudTranslate = node.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
        mTFPointCloudRotate = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);

        mPosePCL = node.getTopicMessageFactory().newFromType(geometry_msgs.PoseStamped._TYPE);
        mPosePCLHeader = node.getTopicMessageFactory().newFromType(Header._TYPE);
        mPosePCLData = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
        mPosePCLTranslation = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
        mPosePCLOrientation = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);

        mPointCloud = node.getTopicMessageFactory().newFromType(sensor_msgs.PointCloud2._TYPE);
        PointStep = 4 * 4; // One field is 4 bytes (FLOAT32), we have 4 fields.
        mPointCloud.setPointStep(PointStep);
        mPointCloudHeader = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);

        PointField mPointFieldX = node.getTopicMessageFactory().newFromType(PointField._TYPE);
        mPointFieldX.setCount(1);
        mPointFieldX.setName("x");
        mPointFieldX.setDatatype(sensor_msgs.PointField.FLOAT32);
        mPointFieldX.setOffset(0);

        PointField mPointFieldY = node.getTopicMessageFactory().newFromType(PointField._TYPE);
        mPointFieldY.setCount(1);
        mPointFieldY.setName("y");
        mPointFieldY.setDatatype(sensor_msgs.PointField.FLOAT32);
        mPointFieldY.setOffset(4);

        PointField mPointFieldZ = node.getTopicMessageFactory().newFromType(PointField._TYPE);
        mPointFieldZ.setCount(1);
        mPointFieldZ.setName("z");
        mPointFieldZ.setDatatype(sensor_msgs.PointField.FLOAT32);
        mPointFieldZ.setOffset(8);

        PointField mPointFieldC = node.getTopicMessageFactory().newFromType(PointField._TYPE);
        mPointFieldC.setCount(1);
        mPointFieldC.setName("confidence");
        mPointFieldC.setDatatype(sensor_msgs.PointField.FLOAT32);
        mPointFieldC.setOffset(12);

        List<PointField> mPointFieldList = new ArrayList<PointField>();
        mPointFieldList.add(mPointFieldX);
        mPointFieldList.add(mPointFieldY);
        mPointFieldList.add(mPointFieldZ);
        mPointFieldList.add(mPointFieldC);
        mPointCloud.setFields(mPointFieldList);

        sequenceNumber = 0;
    }

    //--------------------------------- ROS: Shutdown Node ---------------------------------------//
    void shutdownNode() {
        nodeStarted = false;

        sequenceNumber = 0;
        if (mPointCloudPublisher != null) mPointCloudPublisher.shutdown();
        if (mPosePCLPublisher != null)    mPosePCLPublisher.shutdown();
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

            DataSender dataSender = new DataSender();
            dataSender.execute();
        }

        /*if ((sleep == 0) || ((sleep > 0) && (canSend.get()))) {
            if (sleep > 0) {
                if (canSend.get()) {
                    new Thread(new DataSender2()).start();
                    canSend.set(false);

                    // Setting delay after sleep
                    // Handlers are crashing the app for some wild reason now, weird.
                    // Just creating a handler here will crash an app.
                    //Handler handler = new Handler();

                    // New workaround.
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            int iSleep = (sleep < minSleep) ? minSleep : sleep;
                            try {
                                Thread.sleep(iSleep);
                            } catch (Exception e) {
                                Log.e(TAG, "Exception", e);
                            }
                            canSend.set(true);
                        }
                    }).start();
                }
            } else {
                new Thread(new DataSender2()).start();
            }*/

            changed.set(false);
        }


    //--------------------------------- ROS: Update Parameters -----------------------------------//
    void updateParams() {
        if (node == null) return;

        if (uSleep) {
            uSleep = false;
            ParameterTree params = node.getParameterTree();
            params.set(GraphName.of("/tango_sensors/point_cloud/sleep"), sleep);
        }
        if (uEnabled) {
            uEnabled = false;
            ParameterTree params = node.getParameterTree();
            params.set(GraphName.of("/tango_sensors/point_cloud/enabled"), enabled);
        }
    }
}
