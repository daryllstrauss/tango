/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.experiments.javapointcloud;

import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;
import android.graphics.Point;
import android.media.MediaScannerConnection;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoTextureCameraPreview;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoCameraPreview;
import com.projecttango.tangoutils.Renderer;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Main Activity class for the Point Cloud Sample. Handles the connection to the
 * {@link Tango} service and propagation of Tango XyzIj data to OpenGL and
 * Layout views. OpenGL rendering logic is delegated to the {@link PCRenderer}
 * class.
 */
public class PointCloudActivity extends Activity implements OnClickListener {

    static final String TAG = PointCloudActivity.class.getSimpleName();
    private static final int SECS_TO_MILLISECS = 1000;
    private Tango mTango;
    private TangoConfig mConfig;
    // private TangoCameraPreview tangoCameraPreview;
    private HashMap<Integer, Integer> cameraTextures_;
    private FrameRenderer renderer_;
    private GLSurfaceView view_;

    private PCRenderer mRenderer;
    private GLSurfaceView mGLView;
    private GLSurfaceView mCamView;

    private TextView mDeltaTextView;
    private TextView mPoseCountTextView;
    private TextView mPoseTextView;
    private TextView mQuatTextView;
    private TextView mPoseStatusTextView;
    private TextView mTangoEventTextView;
    private TextView mPointCountTextView;
    private TextView mTangoServiceVersionTextView;
    private TextView mApplicationVersionTextView;
    private TextView mAverageZTextView;
    private TextView mFrequencyTextView;

    private Button mFirstPersonButton;
    private Button mThirdPersonButton;
    private Button mTopDownButton;
    private Button mSaveButton;

    private int count;
    private int mPreviousPoseStatus;
    private float mDeltaTime;
    private float mPosePreviousTimeStamp;
    private float mXyIjPreviousTimeStamp;
    private float mCurrentTimeStamp;
    private String mServiceVersion;
    private boolean mIsTangoServiceConnected;

    private boolean saveScan = false;
    int scanNumber = 1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_jpoint_cloud);
        setTitle(R.string.app_name);

        mPoseTextView = (TextView) findViewById(R.id.pose);
        mQuatTextView = (TextView) findViewById(R.id.quat);
        mPoseCountTextView = (TextView) findViewById(R.id.posecount);
        mDeltaTextView = (TextView) findViewById(R.id.deltatime);
        mTangoEventTextView = (TextView) findViewById(R.id.tangoevent);
        mPoseStatusTextView = (TextView) findViewById(R.id.status);
        mPointCountTextView = (TextView) findViewById(R.id.pointCount);
        mTangoServiceVersionTextView = (TextView) findViewById(R.id.version);
        mApplicationVersionTextView = (TextView) findViewById(R.id.appversion);
        mAverageZTextView = (TextView) findViewById(R.id.averageZ);
        mFrequencyTextView = (TextView) findViewById(R.id.frameDelta);

        mSaveButton = (Button) findViewById(R.id.saveButton);
        mSaveButton.setOnClickListener(this);
        mFirstPersonButton = (Button) findViewById(R.id.first_person_button);
        mFirstPersonButton.setOnClickListener(this);
        mThirdPersonButton = (Button) findViewById(R.id.third_person_button);
        mThirdPersonButton.setOnClickListener(this);
        mTopDownButton = (Button) findViewById(R.id.top_down_button);
        mTopDownButton.setOnClickListener(this);

        mGLView = (GLSurfaceView) findViewById(R.id.gl_surface_view);
        mGLView.setEGLContextClientVersion(2);

        mTango = new Tango(this);
        mConfig = mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        // tangoCameraPreview = (TangoCameraPreview) findViewById(R.id.cameraView);

        int maxDepthPoints = mConfig.getInt("max_point_cloud_elements");
        mRenderer = new PCRenderer(maxDepthPoints);
        mGLView.setRenderer(mRenderer);

        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        mGLView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);

        PackageInfo packageInfo;
        try {
            packageInfo = this.getPackageManager().getPackageInfo(
                    this.getPackageName(), 0);
            mApplicationVersionTextView.setText(packageInfo.versionName);
        } catch (NameNotFoundException e) {
            e.printStackTrace();
        }

        // Display the version of Tango Service
        mServiceVersion = mConfig.getString("tango_service_library_version");
        mTangoServiceVersionTextView.setText(mServiceVersion);
        mIsTangoServiceConnected = false;

        // Set up OpenGL ES surface
        mCamView = new GLSurfaceView(this);
        mCamView.setEGLContextClientVersion(2);
        mCamView.setDebugFlags(GLSurfaceView.DEBUG_CHECK_GL_ERROR);
        mCamView.setRenderer(renderer_ = new FrameRenderer(this));
        mCamView.setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
        mCamView.setOnClickListener(this);
        FrameLayout placeholder = (FrameLayout)findViewById(R.id.cameraFrame);
        placeholder.addView(mCamView);
        cameraTextures_ = new HashMap<>();
    }

    @Override
    protected void onPause() {
        super.onPause();
        try {
            mTango.disconnect();
            mIsTangoServiceConnected = false;
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!mIsTangoServiceConnected) {
            startActivityForResult(
                    Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
                    Tango.TANGO_INTENT_ACTIVITYCODE);
        }
        Log.i(TAG, "onResumed");
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
            Log.i(TAG, "Triggered");
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, R.string.motiontrackingpermission,
                        Toast.LENGTH_LONG).show();
                finish();
                return;
            }
            try {
                setTangoListeners();
            } catch (TangoErrorException e) {
                Toast.makeText(this, R.string.TangoError, Toast.LENGTH_SHORT)
                        .show();
            } catch (SecurityException e) {
                Toast.makeText(getApplicationContext(),
                        R.string.motiontrackingpermission, Toast.LENGTH_SHORT)
                        .show();
            }
            try {
                mTango.connect(mConfig);
                mIsTangoServiceConnected = true;
            } catch (TangoOutOfDateException e) {
                Toast.makeText(getApplicationContext(),
                        R.string.TangoOutOfDateException, Toast.LENGTH_SHORT)
                        .show();
            } catch (TangoErrorException e) {
                Toast.makeText(getApplicationContext(), R.string.TangoError,
                        Toast.LENGTH_SHORT).show();
            }
            // tangoCameraPreview.connectToTangoCamera(mTango,TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
            setUpExtrinsics();
            // tangoCameraPreview.bringToFront();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
        case R.id.first_person_button:
            mRenderer.setFirstPersonView();
            break;
        case R.id.third_person_button:
            mRenderer.setThirdPersonView();
            break;
        case R.id.top_down_button:
            mRenderer.setTopDownView();
            break;
        case R.id.saveButton:
            saveScan = true;
            break;
        default:
            Log.w(TAG, "Unrecognized button click.");
            return;
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        return mRenderer.onTouchEvent(event);
    }

    private void setUpExtrinsics() {
        // Set device to imu matrix in Model Matrix Calculator.
        TangoPoseData device2IMUPose = new TangoPoseData();
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        try {
            device2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
        mRenderer.getModelMatCalculator().SetDevice2IMUMatrix(
                device2IMUPose.getTranslationAsFloats(),
                device2IMUPose.getRotationAsFloats());

        // Set color camera to imu matrix in Model Matrix Calculator.
        TangoPoseData color2IMUPose = new TangoPoseData();

        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
        try {
            color2IMUPose = mTango.getPoseAtTime(0.0, framePair);
        } catch (TangoErrorException e) {
            Toast.makeText(getApplicationContext(), R.string.TangoError,
                    Toast.LENGTH_SHORT).show();
        }
        mRenderer.getModelMatCalculator().SetColorCamera2IMUMatrix(
                color2IMUPose.getTranslationAsFloats(),
                color2IMUPose.getRotationAsFloats());
    }

    private boolean isExternalStorageWritable() {
        String state = Environment.getExternalStorageState();
        Log.i(TAG, "State: "+state);
        boolean v=Environment.MEDIA_MOUNTED.equals(state);
        return v;
    }

    public File getScanStorageDir() {
        // Get the directory for the user's public pictures directory
//        Date d = new Date();
//        String dirName = new SimpleDateFormat("Scans-yyyyMMdd-kkmmss").format(d);
        String dirName = "myScans";
        File path = new File(
                Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS),
                dirName);
        if (!path.exists()) {
            if (!path.mkdirs()) {
                Log.e(TAG, "Directory not created");
                return null;
            }
        }
        return path;
    }

    final protected static char[] hexArray = "0123456789ABCDEF".toCharArray();
    public static String bytesToHex(byte[] bytes, int len) {
        char[] hexChars = new char[len * 2];
        for ( int j = 0; j < len; j++ ) {
            int v = bytes[j] & 0xFF;
            hexChars[j * 2] = hexArray[v >>> 4];
            hexChars[j * 2 + 1] = hexArray[v & 0x0F];
        }
        return new String(hexChars);
    }

    private void saveScanData(TangoPoseData pose, final TangoXyzIjData xyzIj) throws IOException {
        saveScan = false;

        Log.i(TAG, "Saving");
        if (!isExternalStorageWritable()) {
            Log.e(TAG, "External storage unavailable");
            return;
        }
        File dir = getScanStorageDir();
        if (dir == null) {
            Log.e(TAG, "Failed to create scan directory");
            return;
        }
        String filename = String.format("Scan%05d.data", scanNumber);
        scanNumber += 1;
        File file = new File(dir, filename);
        Log.i(TAG, "Writing data to "+file.getAbsolutePath());

        // Set up buffers
        BufferedOutputStream out = new BufferedOutputStream(new FileOutputStream(file));
        ByteBuffer buf = ByteBuffer.allocate(4*8);
        buf.order(ByteOrder.LITTLE_ENDIAN);

        // Write translation
        for (int i=0; i<pose.translation.length; i++) {
            buf.putDouble(pose.translation[i]);
        }
        out.write(buf.array(), 0, 3*8);

        // Write Rotation
        buf.clear();
        for (int i=0; i<pose.rotation.length; i++) {
            buf.putDouble(pose.rotation[i]);
        }
        out.write(buf.array(), 0, 4 * 8);

        // Write field of view
        TangoCameraIntrinsics intrinsics = mTango.getCameraIntrinsics(TangoCameraIntrinsics.TANGO_CAMERA_COLOR);
        double vFOV = 2*Math.atan(0.5*intrinsics.height/intrinsics.fy);
        double hFOV = 2*Math.atan(0.5*intrinsics.width/intrinsics.fx);
        buf.clear();
        buf.putDouble(hFOV);
        buf.putDouble(vFOV);
        out.write(buf.array(), 0, 16);

        // Write points
        buf.clear();
        buf.putInt(xyzIj.xyzCount);
        out.write(buf.array(), 0, 4);
        Log.i(TAG, "xyzCount="+Integer.toString(xyzIj.xyzCount));
        for (int i=0; i<10; i++) {
            Log.i(TAG, Float.toString(xyzIj.xyz.get(i*3))+", "+Float.toString(xyzIj.xyz.get(i*3+1))+", "+Float.toString(xyzIj.xyz.get(i*3+2)));
        }
        for (int i=0; i<xyzIj.xyzCount*3; i++) {
            buf.clear();
            buf.putFloat(xyzIj.xyz.get(i));
            out.write(buf.array(), 0, 4);
        }

        // Write IJ (Not yet implemented in Tango)
        buf.clear();
        int ijCount = xyzIj.ijRows*xyzIj.ijCols;
        Log.i(TAG, "IJCount="+Integer.toString(ijCount));
        Log.i(TAG, "ijRows="+Integer.toString(xyzIj.ijRows)+" ijCols="+Integer.toString(xyzIj.ijCols));
        ijCount=0;
        buf.putInt(ijCount);
        out.write(buf.array(), 0, 4);
//        if (ijCount>0) {
//            byte[] ijBuffer = new byte[ijCount * 4];
//            FileInputStream ijStream = new FileInputStream(
//                    xyzIj.ijParcelFileDescriptor.getFileDescriptor());
//            try {
//                ijStream.read(ijBuffer, 0, ijCount * 4);
//                ijStream.close();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//            out.write(ijBuffer);
//        }
        out.close();

        MediaScannerConnection.scanFile(
                getApplicationContext(),
                new String[] {file.getAbsolutePath()},
                null,
                new MediaScannerConnection.OnScanCompletedListener() {
                    @Override
                    public void onScanCompleted(String path, Uri uri) {
                        Log.v(TAG, "file " + path + " was scanned successfully: " + uri);
                    }
                });

        Log.i(TAG, "Done");
    }

    private void setTangoListeners() {
        // Configure the Tango coordinate frame pair
        final ArrayList<TangoCoordinateFramePair> framePairs = 
                new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        // Attach cameras to textures.
        synchronized(this) {
            for (Map.Entry<Integer, Integer> entry : cameraTextures_.entrySet())
                mTango.connectTextureId(entry.getKey(), entry.getValue());
        }
        // Listen for new Tango data
        mTango.connectListener(framePairs, new OnTangoUpdateListener() {

            @Override
            public void onFrameAvailable(final int cameraId) {
                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                    mCamView.requestRender();
                    // tangoCameraPreview.onFrameAvailable();
                }
            }

            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                mDeltaTime = (float) (pose.timestamp - mPosePreviousTimeStamp)
                        * SECS_TO_MILLISECS;
                mPosePreviousTimeStamp = (float) pose.timestamp;
                if (mPreviousPoseStatus != pose.statusCode) {
                    count = 0;
                }
                count++;
                mPreviousPoseStatus = pose.statusCode;
                mRenderer.getModelMatCalculator().updateModelMatrix(
                        pose.getTranslationAsFloats(),
                        pose.getRotationAsFloats());
                mRenderer.updateViewMatrix();
                mGLView.requestRender();
                // Update the UI with TangoPose information
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        DecimalFormat threeDec = new DecimalFormat("0.000");
                        String translationString = "["
                                + threeDec.format(pose.translation[0]) + ", "
                                + threeDec.format(pose.translation[1]) + ", "
                                + threeDec.format(pose.translation[2]) + "] ";
                        String quaternionString = "["
                                + threeDec.format(pose.rotation[0]) + ", "
                                + threeDec.format(pose.rotation[1]) + ", "
                                + threeDec.format(pose.rotation[2]) + ", "
                                + threeDec.format(pose.rotation[3]) + "] ";

                        // Display pose data on screen in TextViews
                        mPoseTextView.setText(translationString);
                        mQuatTextView.setText(quaternionString);
                        mPoseCountTextView.setText(Integer.toString(count));
                        mDeltaTextView.setText(threeDec.format(mDeltaTime));
                        if (pose.statusCode == TangoPoseData.POSE_VALID) {
                            mPoseStatusTextView.setText(R.string.pose_valid);
                        } else if (pose.statusCode == TangoPoseData.POSE_INVALID) {
                            mPoseStatusTextView.setText(R.string.pose_invalid);
                        } else if (pose.statusCode == TangoPoseData.POSE_INITIALIZING) {
                            mPoseStatusTextView
                                    .setText(R.string.pose_initializing);
                        } else if (pose.statusCode == TangoPoseData.POSE_UNKNOWN) {
                            mPoseStatusTextView.setText(R.string.pose_unknown);
                        }
                    }
                });
            }

            @Override
            public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
//                if(!mRenderer.isValid()) {
//                    return;
//                }
                mCurrentTimeStamp = (float) xyzIj.timestamp;
                final float frameDelta = (mCurrentTimeStamp - mXyIjPreviousTimeStamp)
                        * SECS_TO_MILLISECS;
                mXyIjPreviousTimeStamp = mCurrentTimeStamp;
                try {
                    TangoPoseData pointCloudPose = mTango.getPoseAtTime(
                            mCurrentTimeStamp, framePairs.get(0));

                    mRenderer.getPointCloud().UpdatePoints(xyzIj.xyz);
                    mRenderer.getModelMatCalculator()
                            .updatePointCloudModelMatrix(
                                    pointCloudPose.getTranslationAsFloats(),
                                    pointCloudPose.getRotationAsFloats());
                    mRenderer.getPointCloud().setModelMatrix(
                            mRenderer.getModelMatCalculator()
                                    .getPointCloudModelMatrixCopy());

                    if (saveScan) {
                        saveScanData(pointCloudPose, xyzIj);
                        renderer_.saveFrame();

                    }
                } catch (TangoErrorException e) {
                    Toast.makeText(getApplicationContext(),
                            R.string.TangoError, Toast.LENGTH_SHORT).show();
                } catch (TangoInvalidException e) {
                    Toast.makeText(getApplicationContext(),
                            R.string.TangoError, Toast.LENGTH_SHORT).show();
                } catch (IOException e) {
                }

                // Must run UI changes on the UI thread. Running in the Tango
                // service thread
                // will result in an error.
                runOnUiThread(new Runnable() {
                    DecimalFormat threeDec = new DecimalFormat("0.000");

                    @Override
                    public void run() {
                        // Display number of points in the point cloud
                        mPointCountTextView.setText(Integer
                                .toString(xyzIj.xyzCount));
                        mFrequencyTextView.setText(""
                                + threeDec.format(frameDelta));
                        mAverageZTextView.setText(""
                                + threeDec.format(mRenderer.getPointCloud()
                                        .getAverageZ()));
                    }
                });
            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mTangoEventTextView.setText(event.eventKey + ": "
                                + event.eventValue);
                    }
                });
            }
        });
    }

    public synchronized void attachTexture(final int cameraId, final int textureName) {
        if (textureName > 0) {
            Log.i(TAG, "attachTexture");
            // Link the texture with Tango if the texture changes after
            // Tango is connected. This generally doesn't happen but
            // technically could because they happen in separate
            // threads. Otherwise the link will be made in startTango().
            if (mIsTangoServiceConnected && cameraTextures_.get(cameraId) != textureName) {
                mTango.connectTextureId(cameraId, textureName);
                Log.i(TAG, "connect Texture");
            }
            cameraTextures_.put(cameraId, textureName);
        }
        else
            cameraTextures_.remove(cameraId);
    }

    public Point getCameraFrameSize(int cameraId) {
        TangoCameraIntrinsics intrinsics = mTango.getCameraIntrinsics(cameraId);
        return new Point(intrinsics.width, intrinsics.height);
    }

    public synchronized void updateTexture(int cameraId) {
        if (mIsTangoServiceConnected) {
            try {
                mTango.updateTexture(cameraId);
            }
            catch (TangoInvalidException e) {
                e.printStackTrace();
            }
        }
    }
}
