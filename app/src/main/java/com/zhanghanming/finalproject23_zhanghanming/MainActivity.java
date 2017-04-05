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

    // Sensors
    private Sensor accelerationSensor;
    private Sensor gravitySensor;
    private Sensor mangeticSensor;

    // Listeners
    private AccelerationSensorListener accelerationListener;
    private GravitySensorListener gravitySensorListener;
    private MagneticSensorListener magneticSensorListener;


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


    private double xV;
    private double yV;
    private double xA;
    private double yA;
    private double xOrigin;
    private double yOrigin;

    private double timestamp = 0;

    private float[] gravity;
    private float[] geomagnet;
    private double[] acceleration;

    /* Kalman filter to correct GPS readings with acceleration */
    private KalmanFilter KF;

    /* interval of GPS reading in ms*/
    private final int interval = 4000;

    /* sampling interval of SENSOR_DELAY_GAME is 20ms */
    private final double sensorSamplingInterval = 0.1;

    private final float nanosecond = 1.0f / 1000000000.0f;

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

        gravitySensor = sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);
        gravitySensorListener = new GravitySensorListener();

        mangeticSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        magneticSensorListener = new MagneticSensorListener();

        sensorManager.registerListener(accelerationListener, accelerationSensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(gravitySensorListener, gravitySensor, SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(magneticSensorListener, mangeticSensor, SensorManager.SENSOR_DELAY_GAME);


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

        /* Initialize Kalman Filter */
        KF = new KalmanFilter(new MyProcessModel(), new MyMeasurementModel());

        handler.postDelayed(new Runnable() {
            @Override
            public synchronized void run() {

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
    private synchronized void getXY(double lat, double lng) {
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
        float[] rotationMatrix = new float[9];
        float[] inclineMatrix = new float[9];
        RealVector phoneAcceleration;
        RealMatrix R;

        double dT;

        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }

        @Override
        public synchronized void onSensorChanged(SensorEvent sensorEvent) {

            acceleration = castFloatToDouble(sensorEvent.values);

            phoneAcceleration = MatrixUtils.createRealVector(acceleration);

            SensorManager.getRotationMatrix(rotationMatrix, inclineMatrix, gravity, geomagnet);

            R = MatrixUtils.createRealMatrix(resize3by3(castFloatToDouble(rotationMatrix)));

            phoneAcceleration = R.preMultiply(phoneAcceleration);



            if (timestamp != 0) {
                dT = (sensorEvent.timestamp - timestamp) * nanosecond;

                xV += (xA + phoneAcceleration.getEntry(0)) * dT / 2;
                yV += (yA + phoneAcceleration.getEntry(1)) * dT / 2;


            }

            timestamp = sensorEvent.timestamp;

            xA = phoneAcceleration.getEntry(0);
            yA = phoneAcceleration.getEntry(1);

            // reset velocity when phone is still
            if (Math.abs(xA) < 1.0) xV = 0;
            if (Math.abs(yA) < 1.0) yV = 0;

        }


        private double[][] resize3by3(double[] input) {
            if(input.length != 9) return null;

            double[][] result = new double[3][3];
            int index = 0;

            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result[i][j] = input[index++];
                }
            }

            return result;
        }

        private double[] castFloatToDouble(float[] input) {
            double[] result = new double[input.length];
            int i = 0;
            for(float f : input) {
                result[i++] = (double) f;
            }

            return result;
        }
    }




    private class GravitySensorListener implements SensorEventListener {
        @Override
        public void onAccuracyChanged(Sensor sensor, int i) {

        }

        @Override
        public synchronized void onSensorChanged(SensorEvent sensorEvent) {
            gravity = sensorEvent.values;

        }
    }

    private class MagneticSensorListener implements SensorEventListener {
        @Override
        public synchronized void onSensorChanged(SensorEvent sensorEvent) {
            geomagnet = sensorEvent.values;
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
            double[] diagon = {0.01, 0.01, 0.01, 0.01};
            return MatrixUtils.createRealDiagonalMatrix(diagon);
        }

        @Override
        public RealMatrix getProcessNoise() {
            // assume no process noise first
            double[] processNoise = {0.001, 0.001, 2, 2};
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
        public synchronized RealVector getInitialStateEstimate() {
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
            double[] measurementNoise = {0.2, 0.2, 0.1299673, 0.1199644};
            return MatrixUtils.createRealDiagonalMatrix(measurementNoise);
        }
    }
}
