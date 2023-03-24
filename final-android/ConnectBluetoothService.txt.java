package com.example.androidapp.service;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.os.ParcelUuid;
import android.util.Log;

import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.Charset;
import java.util.UUID;

public class ConnectBluetoothService {
    public static final String TAG = "BtConnectionSvc";
    private static final String appName = "MAP-GRP07-CONTROLLER";
    private static final UUID MY_UUID_INSECURE = UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");
    private final BluetoothAdapter bluetoothAdapter;
    Context context;
    private UUID deviceUUID;
    private ConnectedThread threadConnected;
    public static boolean isConnected = false;
    private AcceptThread insecureAcceptThread;
    private ConnectThread connectThread;
    private BluetoothDevice bluetoothDevice;

    public ConnectBluetoothService(Context context) {
        this.context = context;
        this.bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        start();
    }

    private class AcceptThread extends Thread{
        private final BluetoothServerSocket serverSocket;
        public AcceptThread(){
            BluetoothServerSocket temp = null;
            try {
                temp = bluetoothAdapter.listenUsingInsecureRfcommWithServiceRecord(appName, MY_UUID_INSECURE);
                Log.d(TAG,"Setting up server using "+MY_UUID_INSECURE);
            } catch (IOException e) {
                Log.e(TAG, "AcceptThread: IO Exception "+e.getMessage());
            }
            serverSocket = temp;
        }

        public void run(){
            Log.d(TAG, "AcceptThread: run");
            BluetoothSocket socket = null;
            try{
                Log.d(TAG, "run: RFCOM server socket start.....");
                // Blocking call, returns on a successful connection or an exception only
                // accept a connection from the client and return the socket object
                socket = serverSocket.accept();
                Log.d(TAG, "run: RFCOM server socket accepted connection.");
            }catch (IOException e){
                Log.e(TAG, "AcceptThread: IOException: " + e.getMessage() );
            }

            if(socket != null) {
                connected(socket,bluetoothDevice);
            }
            Log.i(TAG, "END acceptThread ");
        }

        public void cancel() {
            Log.d(TAG, "cancel: Canceling AcceptThread.");
            try {
                serverSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "cancel: Close of AcceptThread ServerSocket failed. " + e.getMessage() );
            }
        }
    }

    private class ConnectThread extends Thread {
        private BluetoothSocket bluetoothSocket;

        public ConnectThread(BluetoothDevice device) {
            Log.d(TAG, "ConnectThread: start");
            bluetoothDevice = device;
        }

        public void run(){
            BluetoothSocket tmp = null;
            Log.i(TAG, "ConnectThread: run");

            for(ParcelUuid uuid:bluetoothDevice.getUuids()){
                deviceUUID = UUID.fromString(uuid.toString());
                Log.d(TAG, "device UUID returned: " + deviceUUID);
                try {
                    Log.d(TAG, "ConnectThread: Trying to create InsecureRfcommSocket using UUID: " + MY_UUID_INSECURE );
                    tmp = bluetoothDevice.createRfcommSocketToServiceRecord(MY_UUID_INSECURE);
                } catch (IOException e) {
                    Log.e(TAG, "ConnectThread: Could not create InsecureRfcommSocket " + e.getMessage());
                }
                bluetoothSocket = tmp;
                bluetoothAdapter.cancelDiscovery();
                try {
                    bluetoothSocket.connect();
                    Log.d(TAG, "run: ConnectThread connected.");
                    break;
                } catch (IOException e) {
                    try {
                        bluetoothSocket.close();
                        Log.d(TAG, "run: Closed Socket.");
                    } catch (IOException e1) {
                        Log.e(TAG, "mConnectThread: run: Unable to close connection in socket " + e1.getMessage());
                    }
                    Log.d(TAG, "run: ConnectThread: Could not connect to UUID: " + MY_UUID_INSECURE );
                }
            }
            connected(bluetoothSocket,bluetoothDevice);
        }
        public void cancel() {
            try {
                Log.d(TAG, "cancel: Closing Client Socket.");
                bluetoothSocket.close();
            } catch (IOException e) {
                Log.e(TAG, "cancel: close() of mmSocket in Connectthread failed. " + e.getMessage());
            }
        }
    }

    public synchronized void start() {
        Log.d(TAG, "start");

        // Cancel any thread attempting to make a connection
        if (connectThread != null) {
            connectThread.cancel();
            connectThread = null;
        }
        if (insecureAcceptThread == null) {
            insecureAcceptThread = new AcceptThread();
            insecureAcceptThread.start();
        }
    }

    public void startClient(BluetoothDevice device){
        Log.d(TAG, "startClient: Started.");

        connectThread = new ConnectThread(device);
        connectThread.start();
    }

    public synchronized void disconnect() {
        if(threadConnected != null){
            threadConnected.cancel();
            threadConnected = null;
        }
        if (connectThread != null) {
            connectThread.cancel();
            connectThread = null;
        }
        if (insecureAcceptThread != null) {
            insecureAcceptThread.cancel();
            insecureAcceptThread = null;
        }

        isConnected = false;
    }

    private class ConnectedThread extends Thread {
        private final BluetoothSocket bluetoothSocket;
        private final InputStream btInStream;
        private final OutputStream btOutStream;

        public ConnectedThread(BluetoothSocket socket) {
            Log.d(TAG, "ConnectedThread: Starting.");

            bluetoothSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;

            try {
                tmpIn = bluetoothSocket.getInputStream();
                tmpOut = bluetoothSocket.getOutputStream();

                forwardIntent("connectionBTStatus","connected");
                isConnected = true;
            } catch (IOException e) {
                e.printStackTrace();
            }

            btInStream = tmpIn;
            btOutStream = tmpOut;
        }

        public void run(){
            byte[] buffer = new byte[1024];
            int bytes;
            while (true) {
                // Read from the InputStream
                try {
                    bytes = btInStream.read(buffer);
                    String incomingMessage = new String(buffer, 0, bytes);
                    Log.d(TAG, "InputStream: " + incomingMessage);
                    handleIncomingBTMessage(incomingMessage);
                } catch (IOException e) {
                    Log.d(TAG, "disconnected while listening to input stream");
                    forwardIntent("connectionBTStatus", "disconnected");
                    isConnected = false;
                    Log.e(TAG, "write: Error reading Input Stream. " + e.getMessage() );
                    break;
                } catch(JSONException e) {
                    Log.e(TAG, "run: JSON Error in handling incomingBTMessage");
                }
            }
        }

        public void write(byte[] bytes) {
            String text = new String(bytes, Charset.defaultCharset());
            Log.d(TAG, "write: Writing to outputstream: " + text);
            try {
                btOutStream.write(bytes);
            } catch (IOException e) {
                Log.e(TAG, "write: Error writing to output stream. " + e.getMessage() );
            }
        }

        public void cancel() {
            try {
                bluetoothSocket.close();
            } catch (IOException e) { }
        }
    }

    private void connected(BluetoothSocket socket, BluetoothDevice device) {
        Log.d(TAG, "connected: Starting.");
        bluetoothDevice = device;
        threadConnected = new ConnectedThread(socket);
        threadConnected.start();
    }

    public void write(byte[] out) {
        Log.d(TAG, "write: Write Called.");
        threadConnected.write(out);
    }

    private void handlePlainTextCommand(String cmd){
        try{
            if (cmd.contains("ROBOT")) {
                String[] commandComponents = cmd.split(",");
                if(commandComponents.length < 4){
                    Log.e(TAG, "handleIncomingBTMessage: The ROBOT plain text command has insufficient parts, command: "+cmd);
                    return;
                }
                int xPos = Integer.parseInt(commandComponents[1]);
                int yPos = Integer.parseInt(commandComponents[2]);
                int dir = -1;
                switch (commandComponents[3].trim().toUpperCase()) {
                    case "N":
                        dir = 0;
                        break;
                    case"E":
                        dir=2;
                        break;
                    case "S":
                        dir=4;
                        break;
                    case"W":
                        dir=6;
                        break;
                }

                JSONObject positionJson = new JSONObject();
                positionJson.put("x",xPos);
                positionJson.put("y",yPos);
                positionJson.put("d",dir);
                forwardIntent("updateRobocarLocation",positionJson.toString());
            }
            if (cmd.contains("TARGET")) {
                String[] commandComponents = cmd.split(",");
                if (commandComponents.length < 3) {
                    Log.e(TAG, "handleIncomingBTMessage: The TARGET plain text command has insufficient parts, command: "+cmd);
                    return;
                }
                JSONObject imageRecResultObj = new JSONObject();
                imageRecResultObj.put("obstacle_id",commandComponents[1]);
                imageRecResultObj.put("image_id",commandComponents[2]);
                forwardIntent("imageResult",imageRecResultObj.toString());
                return;
            }
            if (cmd.contains("STATUS")) {
                String[] getStatus = cmd.split(",");
                forwardIntent("updateRoboCarState",getStatus[1]);
            }
            Log.i(TAG, "handlePlainTextCommand: Unknown Command: "+cmd);
        }catch (Exception e){
            Log.e(TAG, "handleIncomingBTMessage: An error occured while trying to handle plain text cmd");
            e.printStackTrace();
        }
    }

    private void handleIncomingBTMessage(String msg) throws JSONException {
        Log.i(TAG, "handleIncomingBTMessage: New incoming message: "+msg);
        try{
            JSONObject msgJSON = new JSONObject(msg);
            String msgType = msgJSON.getString("cat");
            switch(msgType.toUpperCase()){
                case "INFO":
                    String infoStr = msgJSON.getString("value");
                    forwardIntent("updateRobocarStatus",infoStr);
                    return;
                case "IMAGE-REC":
                    JSONObject imageRecObj = msgJSON.getJSONObject("value");
                    forwardIntent("imageResult",imageRecObj.toString());
                    return;
                case "LOCATION":
                    JSONObject locationObj = msgJSON.getJSONObject("value");
                    forwardIntent("updateRobocarLocation",locationObj.toString());
                    return;
                case "MODE":
                    String mode = msgJSON.getString("value");
                    forwardIntent("updateRobotcarMode",mode);
                    return;
                case "STATUS":
                    String status = msgJSON.getString("value");
                    forwardIntent("updateRoboCarState", status);
            }
        }catch (Exception e){
            JSONObject msgJSON = new JSONObject();
            msgJSON.put("msg",msg);
            forwardIntent("incomingBTMessage", msgJSON.toString());
        }
        handlePlainTextCommand(msg);
    }

    private void forwardIntent(String intentAction, String content){
        Intent sendingIntent = new Intent(intentAction);
        sendingIntent.putExtra("msg", content);
        LocalBroadcastManager.getInstance(context).sendBroadcast(sendingIntent);
    }
}
