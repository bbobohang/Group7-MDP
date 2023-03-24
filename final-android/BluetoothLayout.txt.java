package com.example.androidapp;
import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import com.example.androidapp.service.ConnectBluetoothService;

import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

public class BluetoothLayout extends Fragment {
    private static final String tagForLog = "BluetoothLayout";
    private BluetoothAdapter adapterForBT;
    private boolean isBluetoothActivated;

    // Data
    private HashMap<String, BluetoothDevice> getPairedBTDevices;
    private HashMap<String, BluetoothDevice> getDiscoveredBTDevices;

    // User interface
    private Button getBluetoothButton;
    private Button getSearchButton;
    private BluetoothDiscoveredListViewAdapter discoveredBluetoothAdapter;
    private List<String> getDiscoveredBluetoothAdapterList;
    private BluetoothPairedListViewAdapter pairedBluetoothDevices;
    private List<String> getPairedBluetoothList;


    // Bluetooth
    private ConnectBluetoothService bluetoothConnection;
    private boolean connectionRetry = false;
    private String currentDeviceAddr;
    private Handler reconnectionHandler = new Handler();
    private Button currrentConnectionBtn;
    private boolean initializedBCastReceivers = false;
    Button sendMsgBtn;
    TextView receivedTextView;
    EditText txtMsgToSend;
    TextView storeTxt_discovered_devices;
    private static final String VARIABLE1 = "param1";
    private static final String VARIABLE2 = "param2";
    
    public static BluetoothLayout newInstance(String param1, String param2) {
        BluetoothLayout fragment = new BluetoothLayout();
        Bundle args = new Bundle();
        args.putString(VARIABLE1, param1);
        args.putString(VARIABLE2, param2);
        fragment.setArguments(args);
        return fragment;
    }

    public BluetoothLayout() {
        isBluetoothActivated = false;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getDiscoveredBTDevices = new HashMap<String, BluetoothDevice>();
        getDiscoveredBluetoothAdapterList = new ArrayList<>();
        getPairedBTDevices = new HashMap<String, BluetoothDevice>();
        getPairedBluetoothList = new ArrayList<>();

        IntentFilter btPairingFilter = new IntentFilter(BluetoothDevice.ACTION_BOND_STATE_CHANGED);
        getActivity().registerReceiver(btPairingReceiver, btPairingFilter);

        if(bluetoothConnection == null) bluetoothConnection = new ConnectBluetoothService(getContext());

        if(!initializedBCastReceivers){
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(bluetoothMsgReceiver, new IntentFilter("incomingBTMessage"));
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(sendBluetoothReceiver, new IntentFilter("sendBTMessage"));
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(btConnectionUpdateReceiver, new IntentFilter("connectionBTStatus"));
            initializedBCastReceivers = true;
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.bluetooth_layout, container, false);
        sendMsgBtn = rootView.findViewById(R.id.bluetoothSendMessage);
        receivedTextView = rootView.findViewById(R.id.receivedMessageTextView);
        txtMsgToSend = rootView.findViewById(R.id.bluetoothMessageInput);
        storeTxt_discovered_devices = rootView.findViewById(R.id.bluetoothDiscoveredDevicesTextView);
        
        getSearchButton = rootView.findViewById(R.id.bluetoothSearch);
        getSearchButton.setOnClickListener(v -> {
            searchBluetooth();
        });
        getBluetoothButton = rootView.findViewById(R.id.bluetoothOnOff);
        getBluetoothButton.setOnClickListener(v -> {
            toggleBluetooth();
        });

        sendMsgBtn.setOnClickListener(v -> {
            byte[] bytes = txtMsgToSend.getText().toString().getBytes(Charset.defaultCharset());
            bluetoothConnection.write(bytes);
            txtMsgToSend.setText("");
        });
        
        ListView getPairedBTDevicesListView = rootView.findViewById(R.id.bluetoothPairedDeviceListView);
        pairedBluetoothDevices = new BluetoothPairedListViewAdapter(getContext(), R.layout.bluetooth_paired_device_list, getPairedBluetoothList);
        getPairedBTDevicesListView.setAdapter(pairedBluetoothDevices);
        
        ListView getDiscoveredBTDevicesListView = rootView.findViewById(R.id.bluetoothDeviceListView);
        discoveredBluetoothAdapter = new BluetoothDiscoveredListViewAdapter(getContext(), R.layout.bluetooth_device_list, getDiscoveredBluetoothAdapterList);
        getDiscoveredBTDevicesListView.setAdapter(discoveredBluetoothAdapter);
        initializeBluetooth();
        
        return rootView;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
    }

    @Override
    public void onPause() {
        super.onPause();
        try{
        }catch (Exception e){
            Log.e(tagForLog, "onPause: An error occurred while deregistering receivers");
            e.printStackTrace();
        }
    }

    private void initializeBluetooth() {
        // check if bluetooth is supported on the device
        adapterForBT = BluetoothAdapter.getDefaultAdapter();
        if (adapterForBT == null) {
            showShortToast("Bluetooth not supported");
            return;
        }

        getBluetoothButton.setEnabled(true);
        if (adapterForBT.isEnabled()) isBluetoothActivated = true;

        Set<BluetoothDevice> paired = adapterForBT.getBondedDevices();
        for (BluetoothDevice device : paired) {
            if (!getPairedBTDevices.containsKey(device.getAddress())) {
                getPairedBTDevices.put(device.getAddress(), device);
                getPairedBluetoothList.add(device.getAddress());
                pairedBluetoothDevices.updateList(getPairedBluetoothList);
            }
        }
        updateBluetoothControlButtons();
    }

    private void toggleBluetooth() {
        isBluetoothActivated = !isBluetoothActivated;
        if (isBluetoothActivated) adapterForBT.enable();
        else adapterForBT.disable();
        updateBluetoothControlButtons();
    }

    private void searchBluetooth() {
        if (isBluetoothActivated) {
            getDiscoveredBTDevices.clear();
            discoveredBluetoothAdapter.clear();

            if (adapterForBT.isDiscovering()) adapterForBT.cancelDiscovery();
            // allow device to be discoverable
            Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
            // 300 seconds to discover
            discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
            startActivity(discoverableIntent);

            checkLocationPermission();
            adapterForBT.startDiscovery();

            // get info for each device discovered
            IntentFilter filter = new IntentFilter();
            filter.addAction(BluetoothDevice.ACTION_FOUND);
            filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
            filter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
            getContext().registerReceiver(btDiscoveryReceiver, filter);
            storeTxt_discovered_devices.setVisibility(View.VISIBLE);
        } else {
            showShortToast("Please on your bluetooth");
            Log.d(tagForLog, "Tried to discover without bluetooth enabled");
        }
    }

    private final BroadcastReceiver btDiscoveryReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (BluetoothAdapter.ACTION_DISCOVERY_STARTED.equals(action)) showShortToast("Discovering Devices");
            else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) showShortToast("Discovery Ended");
            else if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (device != null) {
                    String deviceAddress = device.getAddress();
                    // Avoid adding existing devices
                    if (getDiscoveredBTDevices.containsKey(deviceAddress)) return;
                    // Avoid adding devices when its paired
                    if(getPairedBTDevices.containsKey(deviceAddress)) return;
                    getDiscoveredBTDevices.put(deviceAddress, device);
                    getDiscoveredBluetoothAdapterList.add(deviceAddress);
                    discoveredBluetoothAdapter.updateList(getDiscoveredBluetoothAdapterList);
                    Log.d(tagForLog, "Found device: " + device.getName() + ", " + device.getAddress());
                }
            }
        }
    };

    private final BroadcastReceiver btPairingReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            if (action.equals(BluetoothDevice.ACTION_BOND_STATE_CHANGED)) {
                BluetoothDevice mDevice = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                if (mDevice.getBondState() == BluetoothDevice.BOND_BONDED) {
                    Log.d(tagForLog, "BroadcastReceiver: BOND_BONDED.");
                    setPaired(mDevice);
                }
                if (mDevice.getBondState() == BluetoothDevice.BOND_NONE) Log.d(tagForLog, "BroadcastReceiver: BOND_NONE.");
                if (mDevice.getBondState() == BluetoothDevice.BOND_BONDING) Log.d(tagForLog, "BroadcastReceiver: BOND_BONDING.");
            }
        }
    };

    private void updateBluetoothControlButtons() {
        if (!isBluetoothActivated) {
            getBluetoothButton.setText("Bluetooth: OFF");
            getSearchButton.setEnabled(false);
        } else {
            getBluetoothButton.setText("Bluetooth: ON");
            getSearchButton.setEnabled(true);
        }
    }

    public void checkLocationPermission() {
        if (ContextCompat.checkSelfPermission(getContext(), Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED
                && ContextCompat.checkSelfPermission(getContext(), Manifest.permission.ACCESS_COARSE_LOCATION) == PackageManager.PERMISSION_GRANTED) {
            return;
        }
        showShortToast("Please grant your location.");
        requestPermissionLauncher.launch(Manifest.permission.ACCESS_FINE_LOCATION);
        requestPermissionLauncher.launch(Manifest.permission.ACCESS_COARSE_LOCATION);
    }

    private void pairBluetooth(String macAddress) {
        try {
            if (getPairedBTDevices.containsKey(macAddress)) {
                Log.d(tagForLog, "Pair bluetooth: Device " + macAddress + " is already paired");
                return;
            }
            BluetoothDevice device = getDiscoveredBTDevices.get(macAddress);
            if (device == null) {
                Log.d(tagForLog, "Pair bluetooth: Device " + macAddress + " is not found");
                return;
            }

            if (Build.VERSION.SDK_INT > Build.VERSION_CODES.JELLY_BEAN_MR2) {
                Log.d(tagForLog, "Trying to pair with " + macAddress);
                boolean bonded = device.createBond();
                if (!bonded) Log.e(tagForLog, "An error occurred while trying to pair with device " + macAddress);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private ActivityResultLauncher<String> requestPermissionLauncher =
            registerForActivityResult(new ActivityResultContracts.RequestPermission(), isGranted -> {
                if (isGranted) showShortToast("Location permissions granted");
                else {
                }
            });

    private class BluetoothDiscoveredListViewAdapter extends ArrayAdapter<String> {
        private List<String> items;

        public BluetoothDiscoveredListViewAdapter(@NonNull Context context, int resource, @NonNull List<String> objects) {
            super(context, resource, objects);
            items = objects;
        }

        public void updateList(List<String> list) {
            items = list;
            this.notifyDataSetChanged();
        }

        @NonNull
        @Override
        public View getView(int position, @Nullable View convertView, @NonNull ViewGroup parent) {
            if (convertView == null) {
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.bluetooth_device_list, parent, false);
            }
            BluetoothDevice btDevice = getDiscoveredBTDevices.get(items.get(position));

            String deviceName = btDevice.getName();
            String deviceMAC = btDevice.getAddress();

            if (deviceName == null || deviceName.isEmpty()) deviceName = "Unnamed Device";
            if (deviceMAC == null || deviceMAC.isEmpty()) deviceMAC = "No address found";

            TextView btDeviceTitleTxt = convertView.findViewById(R.id.bluetoothTitle);
            TextView btDeviceMACTxt = convertView.findViewById(R.id.bluetoothMacAddress);
            Button btnConnect = convertView.findViewById(R.id.pairBtn);

            btDeviceTitleTxt.setText(deviceName);
            btDeviceMACTxt.setText(deviceMAC);
            btnConnect.setOnClickListener(v -> {
                pairBluetooth(items.get(position));
            });
            return convertView;
        }
    }

    private void setPaired(BluetoothDevice pairedDevice){
        String pairedAddress = pairedDevice.getAddress();
        getPairedBTDevices.put(pairedAddress,pairedDevice);
        getPairedBluetoothList.add(pairedAddress);
        getDiscoveredBTDevices.remove(pairedAddress);
        getDiscoveredBluetoothAdapterList.remove(pairedAddress);
        discoveredBluetoothAdapter.updateList(getDiscoveredBluetoothAdapterList);
        pairedBluetoothDevices.updateList(getPairedBluetoothList);
    }

    private BroadcastReceiver bluetoothMsgReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String text = intent.getStringExtra("msg");
            text = text.replace("msg", "Message:");
            if(receivedTextView.getText() == null || receivedTextView.getText().equals("")){
                receivedTextView.setText("Received messages: \n" + text);
            }else{
                receivedTextView.setText(receivedTextView.getText() + "\n"+text);
            }
        }
    };

    private BroadcastReceiver sendBluetoothReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            String msg = intent.getStringExtra("msg");
            try{
                byte[] msgInBytes = msg.getBytes(Charset.defaultCharset());
                bluetoothConnection.write(msgInBytes);
            }catch(Exception e) {
                Log.e(tagForLog,"An error occurred while sending bluetooth message");
                e.printStackTrace();
            }
        }
    };

    private BroadcastReceiver btConnectionUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            try{
                String status = intent.getStringExtra("msg");
                switch(status.toUpperCase()){
                    case "CONNECTED":
                        currrentConnectionBtn.setText("Disconnect");
                        showShortToast("Successfully Connected");
                        break;
                    case "DISCONNECTED":
                        showShortToast("Bluetooth Connection Disconnected");
                        currrentConnectionBtn.setText("Wait...");
                        if (connectionRetry){
                            Log.d(tagForLog, "retrying connection, set button to wait");
                            reconnectionHandler.postDelayed(reconnectRunnable, 5000);
                        } else {
                            Log.d(tagForLog, "reconnection failed after 50s, btn set to connect");
                            showShortToast("Bluetooth Reconnection Failed");
                            currrentConnectionBtn.setText("Connect");
                        }
                        break;
                }
            }catch (Exception e) {
                Log.e(tagForLog, "An error occurred while reconnecting bluetooth");
                e.printStackTrace();
            }
        }
    };

    private Runnable reconnectRunnable =new Runnable() {
        @Override
        public void run() {
            try {
                if(!ConnectBluetoothService.isConnected && connectionRetry){
                    showShortToast("Reconnecting...");
                    Log.d(tagForLog, "reconnecting device address: " + currentDeviceAddr);
                    connectBluetooth(currentDeviceAddr);
                }
                reconnectionHandler.removeCallbacks(reconnectRunnable);
            }catch (Exception e){
                Log.e(tagForLog,"run: An error occurred while running reconnectRunnable");
                showShortToast("Error reconnecting, retrying in 5s");
                e.printStackTrace();
            }
        }
    };

    private void disconnectBluetooth(){
        bluetoothConnection.disconnect();
    }

    private boolean connectBluetooth(String macAddress) {
        showShortToast("Connecting to: " + macAddress);
        BluetoothDevice btDevice = getPairedBTDevices.get(macAddress);

        if(btDevice == null){
            showShortToast("Bluetooth device not paired");
            //update the connection
            TextView bluetoothStatusTextView = getActivity().findViewById(R.id.bluetoothStatusText);
            String newBluetoothStatus = "Not Available";
            Log.d(tagForLog, "updating bluetooth status at the top: " + btDevice.getName());
            bluetoothStatusTextView.setText(newBluetoothStatus);
            return false;
        }
        try {
            bluetoothConnection.startClient(btDevice);
            currentDeviceAddr = macAddress;
            Log.d(tagForLog, "connectBluetooth device address: " + currentDeviceAddr);
            TextView bluetoothStatusTextView = getActivity().findViewById(R.id.bluetoothStatusText);
            String newBluetoothStatus = btDevice.getName();
            bluetoothStatusTextView.setText(newBluetoothStatus);
            return true;
        } catch(Exception e){
            showShortToast("An error occurred while attempting to start connection");
            e.printStackTrace();
            TextView bluetoothStatusTextView = getActivity().findViewById(R.id.bluetoothStatusText);
            String newBluetoothStatus = "Not Available";
            Log.d(tagForLog, "Updating bluetooth status at the top: " + btDevice.getName());
            bluetoothStatusTextView.setText(newBluetoothStatus);
            return false;
        }
    }

    private void showShortToast(String msg) {
        Toast.makeText(getActivity(), msg, Toast.LENGTH_LONG).show();
    }

    private void forwardIntent(String intentAction, String content){
        Intent sendingIntent = new Intent(intentAction);
        sendingIntent.putExtra("msg", content);
        LocalBroadcastManager.getInstance(getContext()).sendBroadcast(sendingIntent);
    }

    private class BluetoothPairedListViewAdapter extends ArrayAdapter<String> {
        private List<String> items;

        public BluetoothPairedListViewAdapter(@NonNull Context context, int resource, @NonNull List<String> objects) {
            super(context, resource, objects);
            items = objects;
        }

        public void updateList(List<String> list) {
            items = list;
            this.notifyDataSetChanged();
        }

        @NonNull
        @Override
        public View getView(int position, @Nullable View convertView, @NonNull ViewGroup parent) {
            if (convertView == null) convertView = LayoutInflater.from(getContext()).inflate(R.layout.bluetooth_paired_device_list, parent, false);

            BluetoothDevice btDevice = getPairedBTDevices.get(items.get(position));

            String deviceName = btDevice.getName();
            String deviceMAC = btDevice.getAddress();

            if (deviceName == null || deviceName.isEmpty()) {
                deviceName = "Unnamed Device";
            }
            if (deviceMAC == null || deviceMAC.isEmpty()) {
                deviceMAC = "No address found";
            }

            TextView btDeviceTitleTxt = convertView.findViewById(R.id.bluetoothTitle);
            TextView btDeviceMACTxt = convertView.findViewById(R.id.bluetoothMacAddress);
            Button btnConnect = convertView.findViewById(R.id.bluetoothConnect);

            btDeviceTitleTxt.setText(deviceName);
            btDeviceMACTxt.setText(deviceMAC);
            btnConnect.setOnClickListener(v -> {
                if(btnConnect.getText().equals("Disconnect")){
                    connectionRetry = false;
                    disconnectBluetooth();
                    btnConnect.setText("Connect");
                    forwardIntent("updateRoboCarState","finished");

                    // update status of the bluetooth
                    TextView bluetoothStatusTextView = getActivity().findViewById(R.id.bluetoothStatusText);
                    String newBluetoothStatus = "Not Available";
                    Log.d(tagForLog, "updating bluetooth status at the top: ");
                    bluetoothStatusTextView.setText(newBluetoothStatus);
                    return;
                }
                boolean connectSuccess = connectBluetooth(items.get(position));
                if(connectSuccess){
                    btnConnect.setText("Wait..");
                    connectionRetry = true;
                    currrentConnectionBtn = btnConnect;
                }
            });
            return convertView;
        }
    }
}