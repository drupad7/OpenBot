package org.openbot.app.robot.robot;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Looper;
import android.os.SystemClock;
import androidx.annotation.NonNull;
import androidx.core.content.ContextCompat;
import timber.log.Timber;

/** Real GPS fixes from the phone's GPS provider, about once per second. */
public class GpsLocationSource implements RouteLocationSource {

    // Fixes older than this are ignored.
    private static final long MAX_FIX_AGE_MS = 5000L;

    private final Context context;
    private LocationManager locationManager;
    private LocationListener locationListener;

    public GpsLocationSource(Context context) {
        this.context = context.getApplicationContext();
    }

    @Override
    public String start(Listener listener) {
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {
            return ERROR_NO_PERMISSION;
        }

        LocationManager manager = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
        if (manager == null || !manager.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
            return ERROR_GPS_DISABLED;
        }

        stop();

        locationListener =
                new LocationListener() {
                    @Override
                    public void onLocationChanged(@NonNull Location location) {
                        handleLocation(location, listener);
                    }

                    @Override
                    public void onStatusChanged(String provider, int status, Bundle extras) {}

                    @Override
                    public void onProviderEnabled(@NonNull String provider) {}

                    @Override
                    public void onProviderDisabled(@NonNull String provider) {}
                };

        try {
            manager.requestLocationUpdates(
                    LocationManager.GPS_PROVIDER, 1000L, 0f, locationListener, Looper.getMainLooper());
        } catch (SecurityException e) {
            locationListener = null;
            return ERROR_NO_PERMISSION;
        } catch (RuntimeException e) {
            Timber.w(e, "Could not start GPS updates");
            locationListener = null;
            return ERROR_GPS_DISABLED;
        }

        locationManager = manager;
        return null;
    }

    @SuppressWarnings("deprecation")
    private void handleLocation(Location location, Listener listener) {
        long ageMs =
                (SystemClock.elapsedRealtimeNanos() - location.getElapsedRealtimeNanos()) / 1_000_000L;
        if (ageMs > MAX_FIX_AGE_MS) {
            return;
        }

        float accuracy = location.hasAccuracy() ? location.getAccuracy() : Float.MAX_VALUE;
        boolean mock =
                Build.VERSION.SDK_INT >= Build.VERSION_CODES.S
                        ? location.isMock()
                        : location.isFromMockProvider();

        listener.onFix(location.getLatitude(), location.getLongitude(), accuracy, mock);
    }

    @Override
    public void stop() {
        if (locationManager != null && locationListener != null) {
            try {
                locationManager.removeUpdates(locationListener);
            } catch (RuntimeException e) {
                Timber.w(e, "Could not stop GPS updates");
            }
        }
        locationManager = null;
        locationListener = null;
    }

    @Override
    public String getName() {
        return "GPS";
    }

    @Override
    public double getHeadingDeg() {
        return Double.NaN;
    }
}