package com.example.smartguard;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanRecord;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.location.Location;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;
import android.telephony.SmsManager;
import android.util.Log;
import android.webkit.JavascriptInterface;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.UUID;

public class MainActivity extends AppCompatActivity {

    private static final String TAG = "SmartGuard";

    // =========================
    // BLE UUIDs (ESP32 firmware)
    // =========================
    private static final UUID BLE_SERVICE_UUID      = UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    private static final UUID BLE_CHAR_VITALS_UUID  = UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"); // Notify (12-byte BINARY)
    private static final UUID BLE_CHAR_ALERTS_UUID  = UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"); // Notify (<...> framed STRING)
    private static final UUID BLE_CHAR_COMMAND_UUID = UUID.fromString("6E400004-B5A3-F393-E0A9-E50E24DCCA9E"); // Write (A/S)

    private static final UUID CCCD_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    // =========================
    // Permissions request codes
    // =========================
    private static final int REQ_PERMS_BLE = 1001;
    private static final int REQ_PERMS_SMS = 1002;
    private static final int REQ_PERMS_LOC = 1003;

    // =========================
    // SharedPreferences keys
    // =========================
    private static final String PREFS = "smartguard_prefs";
    private static final String KEY_CARETAKER = "caretaker_phone";
    private static final String KEY_LAST_ADDR = "last_ble_addr";

    // =========================
    // UI / WebView
    // =========================
    private WebView myWebView;

    // =========================
    // BLE members
    // =========================
    private BluetoothAdapter bluetoothAdapter;
    private BluetoothLeScanner bleScanner;
    private BluetoothGatt bluetoothGatt;

    private BluetoothGattCharacteristic vitalsChar;
    private BluetoothGattCharacteristic alertsChar;
    private BluetoothGattCharacteristic commandChar;

    private boolean isScanning = false;
    private boolean isConnected = false;

    private final Handler mainHandler = new Handler(Looper.getMainLooper());

    // =========================
    // Caretaker phone (stored)
    // =========================
    private String caretakerPhoneCached = "";

    // =========================
    // Optional: remember last device address
    // =========================
    private String lastBleAddress = "";

    // =========================
    // Scan behavior
    // =========================
    private static final long SCAN_TIMEOUT_MS = 12000;
    private static final String TARGET_NAME_CONTAINS = "cdtp"; // "CDTP Bracelet" gibi

    // =========================
    // Alerts buffering + alarm state
    // - Arduino alerts are framed like: "<SEQ:...,ALERT:FALL>"
    // - NO newline, can be fragmented
    // =========================
    private final StringBuilder alertsBuf = new StringBuilder();
    private boolean alarmActive = false;
    private int lastAlarmSeqSent = -1;

    // =========================
    // Activity lifecycle
    // =========================
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        myWebView = new WebView(this);
        setContentView(myWebView);

        initPrefs();
        initWebView();
        initBluetooth();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        stopScan();
        closeGatt();
        if (myWebView != null) {
            myWebView.destroy();
        }
    }

    // =========================
    // Init: Preferences
    // =========================
    private void initPrefs() {
        SharedPreferences sp = getSharedPreferences(PREFS, MODE_PRIVATE);
        caretakerPhoneCached = sp.getString(KEY_CARETAKER, "");
        lastBleAddress = sp.getString(KEY_LAST_ADDR, "");
    }

    private void saveCaretakerToPrefs(String phoneRaw) {
        caretakerPhoneCached = phoneRaw == null ? "" : phoneRaw.trim();
        SharedPreferences sp = getSharedPreferences(PREFS, MODE_PRIVATE);
        sp.edit().putString(KEY_CARETAKER, caretakerPhoneCached).apply();
    }

    private void saveLastBleAddr(String addr) {
        lastBleAddress = addr == null ? "" : addr.trim();
        SharedPreferences sp = getSharedPreferences(PREFS, MODE_PRIVATE);
        sp.edit().putString(KEY_LAST_ADDR, lastBleAddress).apply();
    }

    // =========================
    // Init: WebView
    // =========================
    @SuppressLint({"SetJavaScriptEnabled"})
    private void initWebView() {
        WebSettings ws = myWebView.getSettings();
        ws.setJavaScriptEnabled(true);
        ws.setDomStorageEnabled(true);
        ws.setAllowFileAccess(true);
        ws.setAllowContentAccess(true);
        ws.setMixedContentMode(WebSettings.MIXED_CONTENT_ALWAYS_ALLOW);

        myWebView.setWebViewClient(new WebViewClient());
        myWebView.addJavascriptInterface(new AndroidBridge(), "Android");

        // Put the HTML file here: app/src/main/assets/health_app.html
        myWebView.loadUrl("file:///android_asset/health_app.html");
    }

    private void jsSendLine(String lineWithOptionalNewline) {
        final String safe = escapeForJs(lineWithOptionalNewline);
        mainHandler.post(() -> {
            if (myWebView == null) return;
            myWebView.evaluateJavascript("window.updateFromAndroid('" + safe + "');", null);
        });
    }

    private static String escapeForJs(String s) {
        if (s == null) return "";
        return s
                .replace("\\", "\\\\")
                .replace("'", "\\'")
                .replace("\n", "\\n")
                .replace("\r", "");
    }

    // =========================
    // Init: Bluetooth
    // =========================
    private void initBluetooth() {
        BluetoothManager bm = (BluetoothManager) getSystemService(Context.BLUETOOTH_SERVICE);
        bluetoothAdapter = (bm != null) ? bm.getAdapter() : null;

        if (bluetoothAdapter == null) {
            Log.e(TAG, "Bluetooth not supported");
            jsSendLine("SEQ:0,STATUS:BT_NOT_SUPPORTED\n");
            return;
        }
        bleScanner = bluetoothAdapter.getBluetoothLeScanner();
        if (bleScanner == null) {
            Log.e(TAG, "BLE scanner is null (BT off?)");
        }
    }

    // =========================
    // Permissions helpers
    // =========================
    private boolean hasBlePermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED &&
                    ContextCompat.checkSelfPermission(this, Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
        } else {
            return ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED;
        }
    }

    private void requestBlePermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            ActivityCompat.requestPermissions(
                    this,
                    new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT},
                    REQ_PERMS_BLE
            );
        } else {
            ActivityCompat.requestPermissions(
                    this,
                    new String[]{Manifest.permission.ACCESS_FINE_LOCATION},
                    REQ_PERMS_BLE
            );
        }
    }

    private boolean hasSmsPermission() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.SEND_SMS) == PackageManager.PERMISSION_GRANTED;
    }

    private void requestSmsPermission() {
        ActivityCompat.requestPermissions(
                this,
                new String[]{Manifest.permission.SEND_SMS},
                REQ_PERMS_SMS
        );
    }

    private boolean hasLocationPermission() {
        return ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                || ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED;
    }

    private void requestLocationPermissionOptional() {
        ActivityCompat.requestPermissions(
                this,
                new String[]{Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.ACCESS_COARSE_LOCATION},
                REQ_PERMS_LOC
        );
    }

    @SuppressLint("MissingPermission")
    private Location getLastKnownLocationSafe() {
        try {
            LocationManager lm = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
            if (lm == null) return null;

            Location best = null;

            if (lm.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
                Location l = lm.getLastKnownLocation(LocationManager.GPS_PROVIDER);
                if (l != null) best = l;
            }
            if (lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
                Location l = lm.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
                if (l != null && (best == null || l.getTime() > best.getTime())) best = l;
            }
            if (lm.isProviderEnabled(LocationManager.PASSIVE_PROVIDER)) {
                Location l = lm.getLastKnownLocation(LocationManager.PASSIVE_PROVIDER);
                if (l != null && (best == null || l.getTime() > best.getTime())) best = l;
            }

            return best;
        } catch (Exception e) {
            Log.e(TAG, "getLastKnownLocationSafe error: " + e.getMessage());
            return null;
        }
    }

    private String buildSmsWithOptionalLocation(String baseMessage) {
        String msg = (baseMessage == null) ? "" : baseMessage.trim();

        // SMS akışını bloklama: izin yoksa sadece istemeyi dene, SMS'i yine gönder
        if (!hasLocationPermission()) {
            try { requestLocationPermissionOptional(); } catch (Exception ignored) {}
            return msg;
        }

        try {
            Location loc = getLastKnownLocationSafe();
            if (loc == null) return msg;

            double lat = loc.getLatitude();
            double lon = loc.getLongitude();

            return msg
                    + "\n\n📍 Konum:"
                    + "\nHarita: https://maps.google.com/?q=" + lat + "," + lon;
        } catch (Exception e) {
            Log.e(TAG, "buildSmsWithOptionalLocation error: " + e.getMessage());
            return msg;
        }
    }

    // =========================
    // WebView Bridge (JS -> Java)
    // =========================
    public class AndroidBridge {

        @JavascriptInterface
        public void toggleConnection() {
            mainHandler.post(() -> {
                if (!hasBlePermissions()) {
                    Log.w(TAG, "BLE permission missing, requesting...");
                    jsSendLine("SEQ:0,STATUS:BLE_PERMISSION_REQUEST\n");
                    requestBlePermissions();
                    return;
                }

                if (!isBluetoothEnabled()) {
                    Log.w(TAG, "Bluetooth is OFF");
                    jsSendLine("SEQ:0,STATUS:BT_OFF\n");
                    return;
                }

                if (isConnected || bluetoothGatt != null) {
                    disconnect();
                } else {
                    // Try last address first if we have it
                    if (lastBleAddress != null && !lastBleAddress.isEmpty()) {
                        BluetoothDevice dev = bluetoothAdapter.getRemoteDevice(lastBleAddress);
                        if (dev != null) {
                            Log.i(TAG, "Trying last device addr first: " + lastBleAddress);
                            connect(dev);
                            return;
                        }
                    }
                    startScan();
                }
            });
        }

        @JavascriptInterface
        public void sendCommand(String c) {
            if (c == null || c.isEmpty()) return;
            char cmd = c.charAt(0);
            mainHandler.post(() -> writeCommand(cmd));
        }

        @JavascriptInterface
        public void saveCaretakerNumber(String phoneRaw) {
            saveCaretakerToPrefs(phoneRaw);
        }

        @JavascriptInterface
        public void sendSms(String message) {
            mainHandler.post(() -> {
                if (!hasSmsPermission()) {
                    requestSmsPermission();
                    return;
                }

                String finalMsg = buildSmsWithOptionalLocation(message);
                sendSmsToCaretaker(finalMsg);
            });
        }
    }

    private boolean isBluetoothEnabled() {
        return bluetoothAdapter != null && bluetoothAdapter.isEnabled();
    }

    // =========================
    // BLE: Scan / Connect
    // =========================
    @SuppressLint("MissingPermission")
    private void startScan() {
        if (bleScanner == null) {
            Log.e(TAG, "startScan: bleScanner null (BT off?)");
            jsSendLine("SEQ:0,STATUS:SCAN_NOT_AVAILABLE\n");
            return;
        }
        if (isScanning) return;

        isScanning = true;

        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build();

        Log.i(TAG, "BLE scan started (NO FILTER). target contains='" + TARGET_NAME_CONTAINS + "'");
        jsSendLine("SEQ:0,STATUS:CONNECTING\n");

        try {
            bleScanner.startScan(null, settings, scanCallback);
        } catch (Exception e) {
            Log.e(TAG, "startScan failed: " + e.getMessage());
            isScanning = false;
            jsSendLine("SEQ:0,STATUS:SCAN_FAILED\n");
            return;
        }

        mainHandler.postDelayed(this::stopScan, SCAN_TIMEOUT_MS);
    }

    @SuppressLint("MissingPermission")
    private void stopScan() {
        if (bleScanner == null) return;
        if (!isScanning) return;

        isScanning = false;
        try {
            bleScanner.stopScan(scanCallback);
        } catch (Exception ignored) {}
        Log.i(TAG, "BLE scan stopped");
    }

    private final ScanCallback scanCallback = new ScanCallback() {

        @SuppressLint("MissingPermission")
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            if (result == null) return;
            BluetoothDevice dev = result.getDevice();
            if (dev == null) return;

            String name = dev.getName();
            String addr = dev.getAddress();

            ScanRecord sr = result.getScanRecord();
            List<ParcelUuid> uuids = (sr != null) ? sr.getServiceUuids() : null;

            Log.i(TAG,
                    "Found: name=" + name +
                            " addr=" + addr +
                            " rssi=" + result.getRssi() +
                            " uuids=" + uuids
            );

            boolean nameMatch = (name != null && name.toLowerCase(Locale.US).contains(TARGET_NAME_CONTAINS));
            boolean uuidMatch = false;
            if (uuids != null) {
                for (ParcelUuid pu : uuids) {
                    if (pu != null && BLE_SERVICE_UUID.equals(pu.getUuid())) {
                        uuidMatch = true;
                        break;
                    }
                }
            }

            if (uuidMatch || nameMatch) {
                stopScan();
                saveLastBleAddr(addr);
                connect(dev);
            }
        }

        @Override
        public void onScanFailed(int errorCode) {
            isScanning = false;
            Log.e(TAG, "Scan failed: " + errorCode);
            jsSendLine("SEQ:0,STATUS:SCAN_FAILED_" + errorCode + "\n");
            jsSendLine("SEQ:0,STATUS:DISCONNECTED\n");
        }
    };

    @SuppressLint("MissingPermission")
    private void connect(BluetoothDevice device) {
        if (device == null) return;
        closeGatt();

        Log.i(TAG, "Connecting to: " + device.getAddress() + " / " + device.getName());

        try {
            bluetoothGatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
        } catch (Exception e) {
            Log.e(TAG, "connectGatt failed: " + e.getMessage());
            setDisconnectedState();
            return;
        }

        jsSendLine("SEQ:0,STATUS:CONNECTING\n");
    }

    @SuppressLint("MissingPermission")
    private void disconnect() {
        stopScan();
        if (bluetoothGatt != null) {
            try {
                bluetoothGatt.disconnect();
            } catch (Exception ignored) {}
        } else {
            setDisconnectedState();
        }
    }

    private void closeGatt() {
        if (bluetoothGatt != null) {
            try { bluetoothGatt.close(); } catch (Exception ignored) {}
        }
        bluetoothGatt = null;
        vitalsChar = null;
        alertsChar = null;
        commandChar = null;
        isConnected = false;

        alarmActive = false;
        lastAlarmSeqSent = -1;
        synchronized (alertsBuf) { alertsBuf.setLength(0); }
    }

    private void setDisconnectedState() {
        closeGatt();
        jsSendLine("SEQ:0,STATUS:DISCONNECTED\n");
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {

        @Override
        public void onConnectionStateChange(@NonNull BluetoothGatt gatt, int status, int newState) {
            Log.i(TAG, "onConnectionStateChange status=" + status + " newState=" + newState);

            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.w(TAG, "GATT error status=" + status + " -> disconnect/close");
                setDisconnectedState();
                return;
            }

            if (newState == BluetoothProfile.STATE_CONNECTED) {
                isConnected = true;
                Log.i(TAG, "GATT connected");
                jsSendLine("SEQ:0,STATUS:CONNECTED\n");

                try {
                    boolean ok = gatt.discoverServices();
                    Log.i(TAG, "discoverServices called: " + ok);
                } catch (Exception e) {
                    Log.e(TAG, "discoverServices failed: " + e.getMessage());
                    setDisconnectedState();
                }

            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                Log.i(TAG, "GATT disconnected");
                setDisconnectedState();
            }
        }

        @Override
        public void onServicesDiscovered(@NonNull BluetoothGatt gatt, int status) {
            Log.i(TAG, "onServicesDiscovered status=" + status);

            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.w(TAG, "Service discovery failed status=" + status);
                setDisconnectedState();
                return;
            }

            BluetoothGattService svc = gatt.getService(BLE_SERVICE_UUID);
            if (svc == null) {
                Log.e(TAG, "Service not found: " + BLE_SERVICE_UUID);
                try {
                    for (BluetoothGattService s : gatt.getServices()) {
                        Log.e(TAG, "Found service: " + s.getUuid());
                    }
                } catch (Exception ignored) {}
                setDisconnectedState();
                return;
            }

            vitalsChar  = svc.getCharacteristic(BLE_CHAR_VITALS_UUID);
            alertsChar  = svc.getCharacteristic(BLE_CHAR_ALERTS_UUID);
            commandChar = svc.getCharacteristic(BLE_CHAR_COMMAND_UUID);

            if (vitalsChar == null || alertsChar == null || commandChar == null) {
                Log.e(TAG, "Characteristic missing. vitals=" + (vitalsChar != null)
                        + " alerts=" + (alertsChar != null)
                        + " cmd=" + (commandChar != null));
                setDisconnectedState();
                return;
            }

            enableNotify(gatt, vitalsChar);
            enableNotify(gatt, alertsChar);

            jsSendLine("SEQ:0,STATUS:READY\n");
        }

        @Override
        public void onCharacteristicChanged(@NonNull BluetoothGatt gatt, @NonNull BluetoothGattCharacteristic characteristic) {
            UUID uuid = characteristic.getUuid();
            byte[] data = characteristic.getValue();
            if (data == null) return;

            if (BLE_CHAR_VITALS_UUID.equals(uuid)) {
                String line = parseVitalsPayload12(data);
                if (line != null && !line.isEmpty()) {
                    if (!line.endsWith("\n")) line += "\n";
                    jsSendLine(line);
                }
            } else if (BLE_CHAR_ALERTS_UUID.equals(uuid)) {
                handleAlertsChunkFramed(data);
            }
        }
    };

    @SuppressLint("MissingPermission")
    private void enableNotify(BluetoothGatt gatt, BluetoothGattCharacteristic ch) {
        if (gatt == null || ch == null) return;

        boolean ok = false;
        try {
            ok = gatt.setCharacteristicNotification(ch, true);
        } catch (Exception e) {
            Log.e(TAG, "setCharacteristicNotification exception: " + e.getMessage());
        }

        if (!ok) {
            Log.e(TAG, "setCharacteristicNotification failed for " + ch.getUuid());
            return;
        }

        BluetoothGattDescriptor cccd = ch.getDescriptor(CCCD_UUID);
        if (cccd == null) {
            Log.e(TAG, "CCCD missing for " + ch.getUuid());
            return;
        }

        try {
            cccd.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
            boolean w = gatt.writeDescriptor(cccd);
            Log.i(TAG, "writeDescriptor(CCCD) for " + ch.getUuid() + " -> " + w);
        } catch (Exception e) {
            Log.e(TAG, "writeDescriptor exception: " + e.getMessage());
        }
    }

    // =========================
    // BLE: Command write
    // =========================
    @SuppressLint("MissingPermission")
    private void writeCommand(char cmd) {
        if (bluetoothGatt == null || commandChar == null) return;

        try {
            commandChar.setValue(new byte[]{(byte) cmd});
            boolean ok = bluetoothGatt.writeCharacteristic(commandChar);
            Log.i(TAG, "writeCharacteristic cmd=" + cmd + " -> " + ok);
        } catch (Exception e) {
            Log.e(TAG, "writeCommand exception: " + e.getMessage());
        }
    }

    // ==========================================================
    // VITALS parsing (NEW): 12-byte BINARY packet from Arduino
    //
    // Packet (12 bytes):
    // [0]=0xA1 header
    // [1..2]=seq (uint16 LE)
    // [3]=bpm (uint8)
    // [4]=spo2 (uint8)
    // [5..6]=g*100 (int16 LE)
    // [7]=active (1/0)
    // [8..9]=inactSec (uint16 LE)
    // [10]=flags (uint8)
    // [11]=checksum XOR (0..10)
    // ==========================================================
    private String parseVitalsPayload12(byte[] data) {
        if (data == null) return "";
        if (data.length != 12) {
            // Eski protokol kalırsa diye: text gibi görünüyorsa geçir
            if (looksLikeTextCsv(data)) {
                return new String(data, StandardCharsets.UTF_8).trim();
            }
            Log.w(TAG, "Vitals unexpected len=" + data.length);
            return "";
        }

        try {
            int header = data[0] & 0xFF;
            if (header != 0xA1) {
                Log.w(TAG, "Vitals bad header=" + header);
                return "";
            }

            // checksum verify
            int cs = 0;
            for (int i = 0; i <= 10; i++) cs ^= (data[i] & 0xFF);
            int recvCs = data[11] & 0xFF;
            if (cs != recvCs) {
                Log.w(TAG, "Vitals checksum mismatch calc=" + cs + " recv=" + recvCs);
                return "";
            }

            ByteBuffer bb = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);

            bb.get(); // header
            int seq = bb.getShort() & 0xFFFF;
            int bpm = bb.get() & 0xFF;
            int spo2 = bb.get() & 0xFF;

            short g100s = bb.getShort(); // signed
            float g = g100s / 100.0f;

            int active = bb.get() & 0xFF;
            boolean isActive = (active != 0);

            int inactSec = bb.getShort() & 0xFFFF;
            int flags = bb.get() & 0xFF;

            // JS tarafına eski style CSV gönderiyoruz (HTML değişmesin diye)
            return "SEQ:" + seq
                    + ",BPM:" + bpm
                    + ",SPO2:" + spo2
                    + ",G:" + String.format(Locale.US, "%.2f", g)
                    + ",ACTIVE:" + (isActive ? 1 : 0)
                    + ",INACT:" + inactSec
                    + ",FLAGS:" + flags;

        } catch (Exception e) {
            Log.e(TAG, "parseVitalsPayload12 exception: " + e.getMessage());
            return "";
        }
    }

    private boolean looksLikeTextCsv(byte[] data) {
        int printable = 0;
        for (byte b : data) {
            int c = b & 0xFF;
            if (c == 10 || c == 13) continue;
            if (c >= 32 && c <= 126) printable++;
        }
        if (printable < Math.max(3, data.length / 2)) return false;

        String s = new String(data, StandardCharsets.UTF_8);
        return s.contains(":") && (s.contains(",") || s.contains("STATUS") || s.contains("ALERT") || s.contains("ALARM"));
    }

    // ==========================================================
    // ALERTS parsing (NEW): framed messages "<...>" with NO newline
    // - can arrive fragmented or multiple frames per chunk
    // ==========================================================
    private void handleAlertsChunkFramed(byte[] data) {
        if (data == null || data.length == 0) return;

        String chunk = new String(data, StandardCharsets.UTF_8);
        if (chunk.isEmpty()) return;

        synchronized (alertsBuf) {
            alertsBuf.append(chunk);

            while (true) {
                int start = alertsBuf.indexOf("<");
                if (start < 0) {
                    // no frame start, prevent unlimited growth
                    if (alertsBuf.length() > 4096) alertsBuf.setLength(0);
                    return;
                }

                int end = alertsBuf.indexOf(">", start + 1);
                if (end < 0) {
                    // wait for rest of frame
                    if (start > 0) alertsBuf.delete(0, start); // drop garbage before '<'
                    return;
                }

                String payload = alertsBuf.substring(start + 1, end).trim();
                alertsBuf.delete(0, end + 1);

                if (!payload.isEmpty()) {
                    handleAlertPayload(payload);
                }
            }
        }
    }

    private void handleAlertPayload(String payload) {
        // UI'ya gönder
        jsSendLine(payload + "\n");

        boolean isManual = payload.contains("ALARM:MANUAL");
        boolean isStop   = payload.contains("ALARM:STOP");

        int seq = extractSeq(payload);

        if (isManual) {
            // ✅ ARTIK OTOMATIK SMS YOK.
            // Sadece state tut: UI overlay açacak, SMS'i Notify caretaker tetikleyecek.
            alarmActive = true;
            lastAlarmSeqSent = seq;
            return;
        }

        if (isStop) {
            alarmActive = false;
        }
    }

    private int extractSeq(String line) {
        if (line == null) return -1;
        int p = line.indexOf("SEQ:");
        if (p < 0) return -1;
        p += 4;

        int end = p;
        while (end < line.length()) {
            char c = line.charAt(end);
            if (c >= '0' && c <= '9') end++;
            else break;
        }
        if (end == p) return -1;

        try {
            return Integer.parseInt(line.substring(p, end));
        } catch (Exception e) {
            return -1;
        }
    }

    // =========================
    // SMS: normalize number + send
    // =========================
    private void sendSmsToCaretaker(String message) {
        String raw = caretakerPhoneCached == null ? "" : caretakerPhoneCached.trim();
        String to = normalizeTrPhone(raw);

        if (to.isEmpty()) {
            jsSendLine("SEQ:0,STATUS:CARETAKER_MISSING\n");
            return;
        }

        try {
            SmsManager sms = SmsManager.getDefault();

            String msg = (message == null) ? "" : message;
            ArrayList<String> parts = sms.divideMessage(msg);

            if (parts != null && parts.size() > 1) {
                sms.sendMultipartTextMessage(to, null, parts, null, null);
            } else {
                sms.sendTextMessage(to, null, msg, null, null);
            }

            Log.i(TAG, "SMS sent to " + to + " parts=" + (parts == null ? 0 : parts.size()));
            jsSendLine("SEQ:0,STATUS:SMS_SENT\n");
        } catch (Exception e) {
            Log.e(TAG, "SMS failed: ", e);
            jsSendLine("SEQ:0,STATUS:SMS_FAILED\n");
        }
    }

    private String normalizeTrPhone(String input) {
        if (input == null) return "";
        String s = input.trim();

        s = s.replace(" ", "")
                .replace("-", "")
                .replace("(", "")
                .replace(")", "");

        if (s.isEmpty()) return "";

        if (s.startsWith("+")) return s;

        if (s.startsWith("90") && s.length() >= 12) return "+" + s;

        if (s.startsWith("0") && s.length() == 11) return "+90" + s.substring(1);

        if (s.startsWith("5") && s.length() == 10) return "+90" + s;

        return s;
    }

    // =========================
    // Permissions result
    // =========================
    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);

        if (requestCode == REQ_PERMS_BLE) {
            if (allGranted(grantResults)) {
                Log.i(TAG, "BLE permission granted");
                startScan();
            } else {
                Log.w(TAG, "BLE perms denied");
                jsSendLine("SEQ:0,STATUS:BLE_PERMISSION_DENIED\n");
                jsSendLine("SEQ:0,STATUS:DISCONNECTED\n");
            }
            return;
        }

        if (requestCode == REQ_PERMS_SMS) {
            if (allGranted(grantResults)) {
                Log.i(TAG, "SMS permission granted");
                jsSendLine("SEQ:0,STATUS:SMS_PERMISSION_GRANTED\n");
            } else {
                Log.w(TAG, "SMS permission denied");
                jsSendLine("SEQ:0,STATUS:SMS_PERMISSION_DENIED\n");
            }
            return;
        }

        if (requestCode == REQ_PERMS_LOC) {
            if (allGranted(grantResults)) {
                Log.i(TAG, "Location permission granted (optional)");
                jsSendLine("SEQ:0,STATUS:LOCATION_PERMISSION_GRANTED\n");
            } else {
                Log.w(TAG, "Location permission denied (optional)");
                jsSendLine("SEQ:0,STATUS:LOCATION_PERMISSION_DENIED\n");
            }
        }
    }

    private boolean allGranted(int[] grantResults) {
        if (grantResults == null || grantResults.length == 0) return false;
        for (int r : grantResults) {
            if (r != PackageManager.PERMISSION_GRANTED) return false;
        }
        return true;
    }
}
