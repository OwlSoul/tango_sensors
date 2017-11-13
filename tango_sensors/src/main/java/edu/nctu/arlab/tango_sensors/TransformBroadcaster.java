package edu.nctu.arlab.tango_sensors;

import android.content.SharedPreferences;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.widget.EditText;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import geometry_msgs.TransformStamped;
import tf2_msgs.TFMessage;

import static android.content.ContentValues.TAG;

@SuppressWarnings("FieldCanBeLocal")
class TransformBroadcaster {
    // Settings.
    private boolean enabled  = true;
    private final long threadSleep = 100;
    // TF2 Transform "/tango_origin"
    private float tf2TX = 0.00f;
    private float tf2TY = 0.00f;
    private float tf2TZ = 0.00f;
    private float tf2RX = 0.00f;
    private float tf2RY = 0.00f;
    private float tf2RZ = 0.00f;
    private float tf2RW = 1.00f;

    final EditText editTextTTX;
    final EditText editTextTTY;
    final EditText editTextTTZ;
    final EditText editTextTRX;
    final EditText editTextTRY;
    final EditText editTextTRZ;
    final EditText editTextTRW;
    //ROS
    private ConnectedNode        node = null;

    Publisher<TFMessage> mTransformPublisher;

    private tf2_msgs.TFMessage             mTFTangoOrigin;
    private geometry_msgs.TransformStamped mTFTangoOriginTransform;
    private geometry_msgs.Transform        mTFTangoOriginTransformData;
    private std_msgs.Header                mTFTangoOriginHeader;
    private geometry_msgs.Vector3          mTangoOriginTranslate;
    private geometry_msgs.Quaternion       mTangoOriginRotate;

    //TangoSensors main class
    private final TangoSensors mTangoSensors;

    TransformBroadcaster(TangoSensors pTangoSensors) {

        mTangoSensors = pTangoSensors;

        //TF2 Translation
        editTextTTX = (EditText) mTangoSensors.findViewById(R.id.editTextTTX);
        editTextTTX.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2TX = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putFloat("/tango_origin/translation/x", tf2TX);
                    editor.apply();
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextTTY = (EditText) mTangoSensors.findViewById(R.id.editTextTTY);
        editTextTTY.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2TY = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putFloat("/tango_origin/translation/y", tf2TY);
                    editor.apply();
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextTTZ = (EditText) mTangoSensors.findViewById(R.id.editTextTTZ);
        editTextTTZ.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2TZ = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putFloat("/tango_origin/translation/z", tf2TZ);
                    editor.apply();
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextTRX = (EditText) mTangoSensors.findViewById(R.id.editTextTRX);
        editTextTRX.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2RX = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putFloat("/tango_origin/rotation/x", tf2RX);
                    editor.apply();
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextTRY = (EditText) mTangoSensors.findViewById(R.id.editTextTRY);
        editTextTRY.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2RY = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putFloat("/tango_origin/rotation/y", tf2RY);
                    editor.apply();
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextTRZ = (EditText) mTangoSensors.findViewById(R.id.editTextTRZ);
        editTextTRZ.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2RZ = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putFloat("/tango_origin/rotation/z", tf2RZ);
                    editor.apply();
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });

        editTextTRW = (EditText) mTangoSensors.findViewById(R.id.editTextTRW);
        editTextTRW.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {

            }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    tf2RW = Float.parseFloat(s.toString());
                    SharedPreferences.Editor editor = mTangoSensors.prefs.edit();
                    editor.putFloat("/tango_origin/rotation/w", tf2RW);
                    editor.apply();
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }

            @Override
            public void afterTextChanged(Editable s) {

            }
        });
    }

    void loadPreferences() {
        tf2TX = mTangoSensors.prefs.getFloat("/tango_origin/translation/x", tf2TX);
        tf2TY = mTangoSensors.prefs.getFloat("/tango_origin/translation/y", tf2TY);
        tf2TZ = mTangoSensors.prefs.getFloat("/tango_origin/translation/z", tf2TZ);
        tf2RX = mTangoSensors.prefs.getFloat("/tango_origin/rotation/x",    tf2RX);
        tf2RY = mTangoSensors.prefs.getFloat("/tango_origin/rotation/y",    tf2RY);
        tf2RZ = mTangoSensors.prefs.getFloat("/tango_origin/rotation/z",    tf2RZ);
        tf2RW = mTangoSensors.prefs.getFloat("/tango_origin/rotation/w",    tf2RW);

        editTextTTX.setText(String.format(Locale.getDefault(),"%.3f", tf2TX));
        editTextTTY.setText(String.format(Locale.getDefault(),"%.3f", tf2TY));
        editTextTTZ.setText(String.format(Locale.getDefault(),"%.3f", tf2TZ));
        editTextTRX.setText(String.format(Locale.getDefault(),"%.3f", tf2RX));
        editTextTRY.setText(String.format(Locale.getDefault(),"%.3f", tf2RY));
        editTextTRZ.setText(String.format(Locale.getDefault(),"%.3f", tf2RZ));
        editTextTRW.setText(String.format(Locale.getDefault(),"%.3f", tf2RW));
    }

    //---------------------------------------- ROS -----------------------------------------------//
    //--------------------------------------------------------------------------------------------//
    private class NodeRunnable implements Runnable {

        private final ConnectedNode node;

        NodeRunnable(ConnectedNode pNode) {
            node = pNode;
        }

        @Override
        public void run() {
            while (enabled) {
                if (mTransformPublisher != null) {
                    mTFTangoOriginHeader.setSeq(0);
                    mTFTangoOriginHeader.setFrameId("/"+ mTangoSensors.mWorldTF);
                    mTFTangoOriginHeader.setStamp(node.getCurrentTime());

                    mTFTangoOriginTransform.setHeader(mTFTangoOriginHeader);
                    mTFTangoOriginTransform.setChildFrameId("/"+ mTangoSensors.mNodeNamespace +"/"+ mTangoSensors.mOdomTF);
                    mTangoOriginTranslate.setX(tf2TX);
                    mTangoOriginTranslate.setY(tf2TY);
                    mTangoOriginTranslate.setZ(tf2TZ);
                    mTangoOriginRotate.setX(tf2RX);
                    mTangoOriginRotate.setY(tf2RY);
                    mTangoOriginRotate.setZ(tf2RZ);
                    mTangoOriginRotate.setW(tf2RW);
                    mTFTangoOriginTransformData.setTranslation(mTangoOriginTranslate);
                    mTFTangoOriginTransformData.setRotation(mTangoOriginRotate);
                    mTFTangoOriginTransform.setTransform(mTFTangoOriginTransformData);

                    List<TransformStamped> mTangoOriginList = new ArrayList<TransformStamped>();
                    mTangoOriginList.add(mTFTangoOriginTransform);
                    mTFTangoOrigin.setTransforms(mTangoOriginList);

                    mTransformPublisher.publish(mTFTangoOrigin);

                    //mTangoOriginList.clear();
                }
                try {
                    Thread.sleep(threadSleep);
                } catch (Exception e) {
                    Log.e(TAG,"Exception",e);
                }
            }
        }
    }

    private NodeRunnable mNodeRunnable;

    //----------------------------------ROS: Set Node --------------------------------------------//
    void setNode(ConnectedNode pNode) {
        this.node = pNode;
    }

    //--------------------------------- ROS: Prepare Node ----------------------------------------//
    void prepareNode(){
        if (node == null) {
            return;
        }

        mTransformPublisher = node.newPublisher("/tf_static", "tf2_msgs/TFMessage");

        mTFTangoOrigin              = node.getTopicMessageFactory().newFromType(tf2_msgs.TFMessage._TYPE);
        mTFTangoOriginTransform     = node.getTopicMessageFactory().newFromType(geometry_msgs.TransformStamped._TYPE);
        mTFTangoOriginTransformData = node.getTopicMessageFactory().newFromType(geometry_msgs.Transform._TYPE);
        mTFTangoOriginHeader        = node.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
        mTangoOriginTranslate       = node.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
        mTangoOriginRotate          = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
    }

    //--------------------------------- ROS: Shutdown Node ---------------------------------------//
    void shutdownNode() {
        enabled = false;
        mTransformPublisher.shutdown();
    }

    //--------------------------------- ROS: Execute Node ----------------------------------------//
    void executeNode(){
        if (node == null) {
            return;
        }
        enabled = true;
        mNodeRunnable = new NodeRunnable(node);
        Thread T = new Thread(mNodeRunnable);
        T.start();
    }
}
