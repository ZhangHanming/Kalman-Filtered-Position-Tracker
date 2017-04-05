package com.zhanghanming.finalproject23_zhanghanming;

import android.app.Activity;
import android.content.Context;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.view.View;

import com.androidplot.xy.EditableXYSeries;
import com.androidplot.xy.LineAndPointFormatter;
import com.androidplot.xy.SimpleXYSeries;
import com.androidplot.xy.XYPlot;

import org.apache.commons.math3.filter.KalmanFilter;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

import java.util.ArrayList;
import java.util.List;

public class MainActivity extends Activity {
    private LocationManager locationManager;
    private LocationListener locationListener;
    private SensorManager sensorManager;
    private Sensor accelerationSensor;
    private Sensor rotationSensor;
    private AccelerationSensorListener accelerationListener;
    private RotationSensorListener rotationListener;

    private XYPlot plot;
    private EditableXYSeries originalSeries;
    private EditableXYSeries correctedSeries;

    private List<Double> originalXVals;
    private List<Double> originalYVals;
    private List<Double> correctedXVals;
    private List<Double> correctedYVals;

    /* in radians*/
    private double lat;
    private double lng;

    /* in meters*/
    private double x;
    private double y;

    /* Angle of rotation of phone around z-axis (pointing to the sky)
    * Assume use hold phone horizontally
    */
    private double zSineRotation;
    private double zCosineRotation;

    private double xV;
    private double yV;
    private double xA;
    private double yA;
    private double xOrigin;
    private double yOrigin;

    /* Kalman filter to correct GPS readings with acceleration */
    private KalmanFilter KF;

    /* interval of GPS reading in ms*/
    private final int interval = 4000;

    /* sampling interval of SENSOR_DELAY_GAME is 20ms */
    private final double sensorSamplingInterval = 0.02;

    private final Handler handler = new Handler();

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        /* assume stationary when app opens*/
        xV = 0D;
        yV = 0D;
        xA = 0D;
        yA = 0D;

        /* Initialize Kalman Filter */
        KF = new KalmanFilter(new MyProcessModel(), new MyMeasurementModel());

        /* Initialize GPS Service*/
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        locationListener = new MyLocationListener();
        try {
            locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 1000, 1, locationListener);
        } catch (SecurityException e) {

        }
        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        /* Initialize Accelerometer */
        accelerationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        accelerationListener = new AccelerationSensorListener();

        /* Initialize Rotation Vector*/
        rotationSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
        rotationListener = new RotationSensorListener();


        sensorManager.registerListener(accelerationListener, accelerationSensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(rotationListener, rotationSensor, SensorManager.SENSOR_DELAY_GAME);

        /* Initialize Graph Plotter*/
        plot = (XYPlot) findViewById(R.id.plot);

        originalXVals = new ArrayList<>();
        originalYVals = new ArrayList<>();
        correctedXVals = new ArrayList<>();
        correctedYVals = new ArrayList<>();

        originalSeries = new SimpleXYSeries(originalXVals, originalYVals, "original reading");
        plot.addSeries(originalSeries, new LineAndPointFormatter(null, Color.RED, null, null));

        correctedSeries = new SimpleXYSeries(correctedXVals, correctedYVals, "corrected reading");
        plot.addSeries(correctedSeries, new LineAndPointFormatter(null, Color.BLUE, null, null));
    }

    public void onClick_StartTracking(View view) {
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {

                  originalXVals.add(x);
                  originalYVals.add(y);

                  double[] measuredState = new double[] {x, y, xV, yV};
                  KF.correct(measuredState);
                  KF.predict();

                  double[] stateEstimate = KF.getStateEstimation();

                  correctedXVals.add(stateEstimate[0]);
                  correctedYVals.add(stateEstimate[1]);

                  redrawSeries();


                  handler.postDelayed(this, interval);

            }
        }, 0);
    }

    public void redrawSeries() {
        plot.removeSeries(originalSeries);
        originalSeries = new SimpleXYSeries(originalXVals, originalYVals, "original reading");
        plot.addSeries(originalSeries, new LineAndPointFormatter(null, Color.RED, null, null));

        plot.removeSeries(correctedSeries);
        correctedSeries = new SimpleXYSeries(correctedXVals, correctedYVals, "corrected reading");
        plot.addSeries(correctedSeries, new LineAndPointFormatter(null, Color.BLUE, null, null));

        plot.redraw();
    }

    public void onClick_SetOrigin(View view) {
            xOrigin = lng;
            yOrigin = lat;
    }

    /* uses radian */
    private void getXY(double lat, double lng) {
        double deltaLat = lat - yOrigin;
        double deltaLng = lng - xOrigin;

        double latCircumference = 40075160 * Math.cos(yOrigin);
        x = deltaLng * latCircumference / 2 / Math.PI;
        y = deltaLat * 40008000 / 2 / Math.PI;
    }

    private class MyLocationListener implements LocationListener {
        @Override
        public void onStatusChanged(String s, int i, Bundle bundle) {

        }

        @Override
        public void onProviderEnabled(String s) {

        }

        @Override
        public void onProviderDisabled(String s) {

        }

        @Override
        public void onLocationChanged(Location location) {
            lng = Math.toRadians(location.getLongitude());
            lat = Math.toRadians(location.getLatitude());

            getXY(lat, lng);
        }
    }

    private class AccelerationSensorListener implements SensorEventListener {
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }

        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            // Use Trapezoidal Rule to integrate acceleration
            xV += (xA + sensorEvent.values[0]) * sensorSamplingInterval / 2;
            yV += (yA + sensorEvent.values[1]) * sensorSamplingInterval / 2;

            // correct phone rotation
            xA = sensorEvent.values[0] * zCosineRotation + sensorEvent.values[1] * zSineRotation;
            yA = sensorEvent.values[1] * zCosineRotation + sensorEvent.values[0] * zSineRotation;
        }
    }

    private class RotationSensorListener implements SensorEventListener {
        @Override
        public void onSensorChanged(SensorEvent sensorEvent) {
            zSineRotation = Math.sin(2*Math.asin(sensorEvent.values[2]));
            zCosineRotation = Math.sqrt(1 - Math.pow(zSineRotation, 2));
        }

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }
    }
    private class MyProcessModel implements ProcessModel {


        @Override
        public RealMatrix getControlMatrix() {
            double[] control = {0, 0, 0, 0};
            return MatrixUtils.createRealDiagonalMatrix(control);
        }

        @Override
        public RealMatrix getInitialErrorCovariance() {
            return MatrixUtils.createRealIdentityMatrix(4);
        }

        @Override
        public RealMatrix getProcessNoise() {
            // assume no process noise first
            double[] processNoise = {0.1, 0.1, 0.1, 0.1};
            return MatrixUtils.createRealDiagonalMatrix(processNoise);
        }

        @Override
        public RealMatrix getStateTransitionMatrix() {
            /* assume constant velocity during interval*/
            double[][] stateTransitionMatrixData = {
                    {1, 0, interval / 1000.0, 0},
                    {0, 1, 0, interval / 1000.0},
                    {0, 0, 1, 0},
                    {0, 0, 0, 1}
            };
            return MatrixUtils.createRealMatrix(stateTransitionMatrixData);
        }

        @Override
        public RealVector getInitialStateEstimate() {
            double[] initialStateEstimate = {x, y, xV, yV};
            return MatrixUtils.createRealVector(initialStateEstimate);
        }
    }
    private class MyMeasurementModel implements MeasurementModel {
        @Override
        public RealMatrix getMeasurementMatrix() {
            return MatrixUtils.createRealIdentityMatrix(4);
        }

        @Override
        public RealMatrix getMeasurementNoise() {
            // TODO replace mesaurementNoise with values
            double[] measurementNoise = {0.1, 0.1, 0.1299673, 0.1199644};
            return MatrixUtils.createRealDiagonalMatrix(measurementNoise);
        }
    }
}
