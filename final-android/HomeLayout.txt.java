package com.example.androidapp;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import org.json.JSONArray;
import org.json.JSONObject;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

public class HomeLayout extends Fragment{
    public static String TAG = "HomeLayout";

    // setting arena flags
    boolean settingRobot, settingObstacle, settingDir;
    // arena related UI
    Button sendArenaBtn;
    Button startImageRecBtn;
    Button startFastestBtn;

    // gridmap related
    private static GridArena gridMap;
    // Map related UI
    Button resetArenaBtn;

    // robot related
    private TextView robotStatusTextView;
    // robot related UI
    Button placeRobotBtn;

    // obstacle related
    private ObstaclesListViewAdapter obstaclesListViewAdapter;
    private List<ObstacleListItem> obstacleListItemList;
    // obstacle related UI
    Button setObstacleBtn;
    Button setFacingBtn;
    Button btnAddObsManual;
    EditText addObsXCoord;
    EditText addObsYCoord;

    // rootview
    private View rootview;

    private long timeStarted;
    private long timeEnded;
    private long timeTakenInNanoSeconds;
    TextView timeTakenTextView;

    // flags
    private boolean initializedIntentListeners = false;

    // fragment initialization parameters, e.g. ARG_ITEM_NUMBER
    private static final String ARG_PARAM1 = "param1";
    private static final String ARG_PARAM2 = "param2";
    private String mParam1;
    private String mParam2;

    public HomeLayout() {
        // Required empty public constructor
    }

    /**
     * Use this factory method to create a new instance of
     * this fragment using the provided parameters.
     *
     * @param param1 Parameter 1.
     * @param param2 Parameter 2.
     * @return A new instance of fragment ArenaFragment.
     */
    public static HomeLayout newInstance(String param1, String param2) {
        HomeLayout fragment = new HomeLayout();
        Bundle args = new Bundle();
        args.putString(ARG_PARAM1, param1);
        args.putString(ARG_PARAM2, param2);
        fragment.setArguments(args);
        return fragment;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        
        // list of obstacle list item objects
        obstacleListItemList = new ArrayList<>();

        if (getArguments() != null) {
            mParam1 = getArguments().getString(ARG_PARAM1);
            mParam2 = getArguments().getString(ARG_PARAM2);
        }

        if(!initializedIntentListeners){
            // register to receive messages
            // registering an observer (roboStatusUpdateReceiver) to receive Intents with actions named "updateRobocarStatus"
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(robocarStatusUpdateReceiver, new IntentFilter("updateRobocarStatus"));
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(robocarStateUpdateReceiver, new IntentFilter("updateRoboCarState"));
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(obstacleListUpdateReceiver, new IntentFilter("newObstacleList"));
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(imageRecResultReceiver, new IntentFilter("imageResult"));
            LocalBroadcastManager.getInstance(getContext()).registerReceiver(robocarLocationUpdateReceiver, new IntentFilter("updateRobocarLocation"));

            initializedIntentListeners = true;
        }
    }

    // onCreateView to inflate layout of fragment
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        rootview = inflater.inflate(R.layout.home_layout, container, false);

        if(gridMap == null){
            gridMap = new GridArena(getContext());
            gridMap = rootview.findViewById(R.id.gridView);
        }

        //For obstacle list view
        ListView obstacleListView = (ListView)  rootview.findViewById(R.id.obstaclesListView);
        obstaclesListViewAdapter = new ObstaclesListViewAdapter(getContext(), R.layout.obstacle_list_item_layout, obstacleListItemList);
        obstacleListView.setAdapter(obstaclesListViewAdapter);

        // robot related
        // initialise settingRobot flag
        settingRobot = false;
        // for updating of robot status
        this.robotStatusTextView = (TextView) rootview.findViewById(R.id.robotStatusText);

        // declaring control buttons
        ImageButton controlBtnUp = rootview.findViewById(R.id.upArrowBtn);
        ImageButton controlBtnDown = rootview.findViewById(R.id.downArrowBtn);
        ImageButton controlBtnLeft = rootview.findViewById(R.id.leftArrowBtn);
        ImageButton controlBtnRight = rootview.findViewById(R.id.rightArrowBtn);
        // onTouch event handlers to detect when user releases the control button
        // up control button onTouch event handler
        controlBtnUp.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                // detect when user presses the control button
                if(event.getAction() == MotionEvent.ACTION_DOWN) {
                    sendDirCmdIntent("FW--");

                // detect when user releases the control button
                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    sendDirCmdIntent("STOP");
                }

                return true;
            }
        });

        // down or reverse control button onTouch event handler
        controlBtnDown.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if(event.getAction() == MotionEvent.ACTION_DOWN) {
                    sendDirCmdIntent("BW--");

                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    sendDirCmdIntent("STOP");
                }

                return true;
            }
        });

        // left control button onTouch event handler
        controlBtnLeft.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if(event.getAction() == MotionEvent.ACTION_DOWN) {
                    sendDirCmdIntent("TL--");

                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    sendDirCmdIntent("STOP");
                }

                return true;
            }
        });

        // right control button onTouch event handler
        controlBtnRight.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if(event.getAction() == MotionEvent.ACTION_DOWN) {
                    sendDirCmdIntent("TR--");

                } else if (event.getAction() == MotionEvent.ACTION_UP) {
                    sendDirCmdIntent("STOP");
                }
                return true;
            }
        });

        // intialise arena related buttons
        resetArenaBtn = rootview.findViewById(R.id.resetArenaBtn);
        setObstacleBtn = rootview.findViewById(R.id.setObstacleBtn);
        setFacingBtn = rootview.findViewById(R.id.setFacingBtn);
        placeRobotBtn = rootview.findViewById(R.id.placeRobotBtn);

        // intialise robot related buttons
        sendArenaBtn = rootview.findViewById(R.id.sendArenaBtn);
        startImageRecBtn = rootview.findViewById(R.id.startImageRecBtn);
        startFastestBtn = rootview.findViewById(R.id.startFastestBtn);

        // intialise obstacle related buttons
        btnAddObsManual = rootview.findViewById(R.id.addObsBtnManual);
        addObsXCoord = rootview.findViewById(R.id.add_obs_x_coord);
        addObsYCoord = rootview.findViewById(R.id.add_obs_y_coord);

        // OnClickListeners for the arena related buttons
        resetArenaBtn.setOnClickListener(v->{
            try{
                gridMap.resetGrid();
            }catch (Exception e){
                Log.e(TAG, "onCreateView: An error occured while resetting map");
                e.printStackTrace();
            }
        });

        setObstacleBtn.setOnClickListener(v->{
            try{
                settingObstacle = !settingObstacle;
                if(settingObstacle){
                    gridMap.setSetObstacleStatus(settingObstacle);
                    setObstacleBtn.setText("Stop Set Obstacle");

                    //Disable other buttons
                    setFacingBtn.setEnabled(false);
                    placeRobotBtn.setEnabled(false);
                    resetArenaBtn.setEnabled(false);
                    startFastestBtn.setEnabled(false);
                    startImageRecBtn.setEnabled(false);
                }else{
                    gridMap.setSetObstacleStatus(settingObstacle);
                    setObstacleBtn.setText("Set Obstacle");

                    //Re-enable other buttons
                    setFacingBtn.setEnabled(true);
                    placeRobotBtn.setEnabled(true);
                    resetArenaBtn.setEnabled(true);
                    startFastestBtn.setEnabled(true);
                    startImageRecBtn.setEnabled(true);
                }
            }catch (Exception e){
                Log.e(TAG, "onCreateView: An error occurred while setting obstacle");
                e.printStackTrace();
            }
        });

        setFacingBtn.setOnClickListener(v -> {
            try{
                settingDir = !settingDir;
                if(settingDir){
                    gridMap.setSetObstacleDirection(settingDir);
                    setFacingBtn.setText("Stop Set Facing");

                    //Disable Other Buttons
                    setObstacleBtn.setEnabled(false);
                    placeRobotBtn.setEnabled(false);
                    resetArenaBtn.setEnabled(false);
                    startFastestBtn.setEnabled(false);
                    startImageRecBtn.setEnabled(false);
                }else{
                    gridMap.setSetObstacleDirection(settingDir);
                    setFacingBtn.setText("Set Facing");

                    //Reenable other buttons
                    setObstacleBtn.setEnabled(true);
                    placeRobotBtn.setEnabled(true);
                    resetArenaBtn.setEnabled(true);
                    startFastestBtn.setEnabled(true);
                    startImageRecBtn.setEnabled(true);
                }
            }catch (Exception e){
                Log.e(TAG, "onCreateView: An error occurred while setting obstacle direction");
                e.printStackTrace();
            }
        });

        placeRobotBtn.setOnClickListener(v -> {
            try{
                //New status
                settingRobot = !settingRobot;
                if(settingRobot){
                    gridMap.setStartCoordStatus(settingRobot);
                    placeRobotBtn.setText("Stop Set Robot");

                    //Disable other buttons
                    setObstacleBtn.setEnabled(false);
                    setFacingBtn.setEnabled(false);
                    resetArenaBtn.setEnabled(false);
                    startFastestBtn.setEnabled(false);
                    startImageRecBtn.setEnabled(false);
                }else{
                    gridMap.setStartCoordStatus(settingRobot);
                    setObstacleBtn.setEnabled(true);
                    setFacingBtn.setEnabled(true);
                    resetArenaBtn.setEnabled(true);
                    startFastestBtn.setEnabled(true);
                    startImageRecBtn.setEnabled(true);
                    placeRobotBtn.setText("Place Robot");
                }
            }catch (Exception e){
                Log.e(TAG, "onCreateView: An error occured while placing robot");
                e.printStackTrace();
            }
        });

        // OnClickListeners for robot related buttons, to send arena info to RPI
        sendArenaBtn.setOnClickListener(v->{
            Log.d(TAG, "Send Arena Clicked");
            // send obstacle info in JSONObject to RPI
            gridMap.sendUpdatedObstacleInformation();
        });

        startImageRecBtn.setOnClickListener(v->{
            Log.d(TAG, "Start Image Rec Clicked");
            // send obstacle info in JSONObject to RPI
            gridMap.removeAllTargetIDs();
            gridMap.sendUpdatedObstacleInformation();
            new Timer().schedule(new TimerTask() {
                @Override
                public void run() {
                    sendControlCmdIntent("stop");
                }
            }, 360000);

        });

        startFastestBtn.setOnClickListener(v->{
            sendControlCmdIntent("start");
        });

        // OnClickListeners for the obstacle related buttons
        btnAddObsManual.setOnClickListener(v -> {
            try{
                String x_value = addObsXCoord.getText().toString();
                String y_value = addObsYCoord.getText().toString();
                try
                {
                    int x_value_int = Integer.parseInt(x_value);
                    int y_value_int = Integer.parseInt(y_value);

                    if( x_value_int < 20 && x_value_int >=0 && y_value_int < 20 && y_value_int >=0){
                        gridMap.setObstacleCoord(x_value_int, y_value_int);
                        showShortToast("Added obstacle");
                        addObsXCoord.setText("");
                        addObsYCoord.setText("");
                    }else{
                        showShortToast("Invalid Coordinates");
                    }
                }catch (Exception e){
                    showShortToast("Incorrect values!");
                }
            }catch (Exception e){
                Log.e(TAG, "onCreateView: An error occurred while adding obstacle manually");
                e.printStackTrace();
            }
        });


        // Inflate the layout for this fragment
        return rootview;
    }

    // handler for received Intents. This will be called whenever an Intent with an action named "updateRobotcarStatus" is broadcasted.
    private BroadcastReceiver robocarStatusUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            try{
                String msgInfo = intent.getStringExtra("msg");
                robotStatusTextView.setText(msgInfo);
            }catch (Exception e){
                robotStatusTextView.setText("UNKNOWN");
                showShortToast("Error updating robocar status");
                Log.e(TAG, "onReceive: An error occured while updating the robocar status");
                e.printStackTrace();
            }
        }
    };

    private BroadcastReceiver robocarStateUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            try{
                String state = intent.getStringExtra("msg");
                updateRobotStatus(state);
                switch(state.toUpperCase()){
                    case "FINISHED":
                        timeEnded = System.nanoTime();
                        timeTakenInNanoSeconds = timeEnded - timeStarted;

                        double timeTakenInSeconds = (double) timeTakenInNanoSeconds/1000000000;
                        int timeTakenMin = (int) timeTakenInSeconds/60;
                        double timeTakenSec = (double) timeTakenInSeconds%60;
                        DecimalFormat df = new DecimalFormat("0.00");

                        timeTakenTextView.setText("Run completed in: "+Integer.toString(timeTakenMin)+"min "+df.format(timeTakenSec)+"secs");
                        timeTakenTextView.setVisibility(View.VISIBLE);
                        setObstacleBtn.setEnabled(true);
                        placeRobotBtn.setEnabled(true);
                        resetArenaBtn.setEnabled(true);
                        setFacingBtn.setEnabled(true);
                        startFastestBtn.setEnabled(true);
                        startImageRecBtn.setEnabled(true);
                        sendArenaBtn.setEnabled(true);
                        btnAddObsManual.setEnabled(true);
                        break;
                    case "RUNNING":
                        setObstacleBtn.setEnabled(false);
                        placeRobotBtn.setEnabled(false);
                        resetArenaBtn.setEnabled(false);
                        setFacingBtn.setEnabled(false);
                        startFastestBtn.setEnabled(false);
                        startImageRecBtn.setEnabled(false);
                        sendArenaBtn.setEnabled(false);
                        btnAddObsManual.setEnabled(false);
                        break;
                }
            }catch (Exception ex){
                Log.e(TAG, "onReceive: Error receiving robot completion status");
            }
        }
    };

    private BroadcastReceiver obstacleListUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            obstacleListItemList.clear();
            try{
                JSONArray msgInfo = new JSONArray(intent.getStringExtra("msg"));
                for(int i=0; i<msgInfo.length();i++){
                    JSONObject obj = msgInfo.getJSONObject(i);
                    obstacleListItemList.add(new ObstacleListItem(obj.getInt("no"), obj.getInt("x"),obj.getInt("y"),obj.getString("facing")));
                }
                obstaclesListViewAdapter.updateList(obstacleListItemList);
            }catch (Exception ex){
                Log.e(TAG, "onReceive: An error occured while updating obstacle list view");
                ex.printStackTrace();
            }
        }
    };

    private BroadcastReceiver imageRecResultReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            try{
                JSONObject msgJSON = new JSONObject(intent.getStringExtra("msg"));
                int obstacleID = Integer.parseInt(msgJSON.getString("obstacle_id"));
                String targetID = msgJSON.getString("image_id");
                gridMap.updateObsImageID(obstacleID, targetID);
            }catch (Exception e){
                showShortToast("Error updating image rec result");
                Log.e(TAG, "onReceive: An error occured while upating the image rec result");
                e.printStackTrace();
            }
        }
    };

    private BroadcastReceiver robocarLocationUpdateReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            try{
                JSONObject msgJSON = new JSONObject(intent.getStringExtra("msg"));
                int xCoord = msgJSON.getInt("x");
                int yCoord = msgJSON.getInt("y");
                int dirInt = msgJSON.getInt("d");
                GridArena.Direction direction = GridArena.Direction.UP;
                switch(dirInt){
                    case 0: //NORTH
                        direction = GridArena.Direction.UP;
                        break;
                    case 2: //EAST
                        direction = GridArena.Direction.RIGHT;
                        break;
                    case 4: //SOUTH
                        direction = GridArena.Direction.DOWN;
                        break;
                    case 6: //WEST
                        direction = GridArena.Direction.LEFT;
                        break;
                }

                if(xCoord < 0 || yCoord < 0 || xCoord > 20 || yCoord > 20){
                    showShortToast("Error: Robot move out of area (x: "+xCoord+", y: "+yCoord+")");
                    Log.e(TAG, "onReceive: Robot is out of the arena area");
                    return;
                }
                gridMap.updateCurCoord(xCoord, yCoord, direction);
                updateCoordinates(xCoord, yCoord, direction);
            }catch (Exception e){
                showShortToast("Error updating robot location");
                Log.e(TAG, "onReceive: An error occured while updating robot location");
                e.printStackTrace();
            }
        }
    };

    private void updateRobotStatus(String status) {
        TextView robotStatusTextView = ((Activity) this.getContext()).findViewById(R.id.robotStatusText);
        robotStatusTextView.setText(status.toUpperCase());
    }

    private void updateCoordinates(int mapX, int mapY, GridArena.Direction direction) {
        // update the robot X Y coordinates
        TextView xAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_x_coord);
        TextView yAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_y_coord);
        String newDirText_x = "X: " + String.valueOf(mapX);
        String newDirText_y = "Y: " + String.valueOf(mapY);
        TextView directionAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robotDirText);
        directionAxisTextView.setText("DIR: "+ String.valueOf(direction));
        xAxisTextView.setText(newDirText_x);
        yAxisTextView.setText(newDirText_y);
    }

    private void sendDirCmdIntent(String direction){

        try{
            JSONObject directionJSONObj = new JSONObject();
            directionJSONObj.put("cat","manual");
            directionJSONObj.put("value",direction);

            // moving robot on gridmap
            if (direction == "FW--") {
                if (gridMap.getRobotFacingDir() == GridArena.Direction.UP) {
                    gridMap.updateCurCoord(gridMap.getXCoord(), gridMap.getYCoord() + 1, gridMap.getRobotFacingDir());
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.DOWN) {
                    gridMap.updateCurCoord(gridMap.getXCoord(), gridMap.getYCoord() - 1, gridMap.getRobotFacingDir());
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.LEFT) {
                    gridMap.updateCurCoord(gridMap.getXCoord() - 1, gridMap.getYCoord(), gridMap.getRobotFacingDir());
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.RIGHT) {
                    gridMap.updateCurCoord(gridMap.getXCoord() + 1, gridMap.getYCoord(), gridMap.getRobotFacingDir());
                }
            }
            else if (direction == "BW--") {
                if (gridMap.getRobotFacingDir() == GridArena.Direction.UP) {
                    gridMap.updateCurCoord(gridMap.getXCoord(), gridMap.getYCoord() - 1, gridMap.getRobotFacingDir());
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.DOWN) {
                    gridMap.updateCurCoord(gridMap.getXCoord(), gridMap.getYCoord() + 1, gridMap.getRobotFacingDir());
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.LEFT) {
                    gridMap.updateCurCoord(gridMap.getXCoord() + 1, gridMap.getYCoord(), gridMap.getRobotFacingDir());
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.RIGHT) {
                    gridMap.updateCurCoord(gridMap.getXCoord() - 1, gridMap.getYCoord(), gridMap.getRobotFacingDir());
                }
            }
            else if (direction == "TL--") {
                if (gridMap.getRobotFacingDir() == GridArena.Direction.UP) {
                    gridMap.setRobotFacingDir(GridArena.Direction.LEFT);
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.DOWN) {
                    gridMap.setRobotFacingDir(GridArena.Direction.RIGHT);
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.LEFT) {
                    gridMap.setRobotFacingDir(GridArena.Direction.DOWN);
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.RIGHT) {
                    gridMap.setRobotFacingDir(GridArena.Direction.UP);
                }
            }
            else if (direction == "TR--") {
                if (gridMap.getRobotFacingDir() == GridArena.Direction.UP) {
                    gridMap.setRobotFacingDir(GridArena.Direction.RIGHT);
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.DOWN) {
                    gridMap.setRobotFacingDir(GridArena.Direction.LEFT);
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.LEFT) {
                    gridMap.setRobotFacingDir(GridArena.Direction.UP);
                }
                else if (gridMap.getRobotFacingDir() == GridArena.Direction.RIGHT) {
                    gridMap.setRobotFacingDir(GridArena.Direction.DOWN);
                }
            }

            broadcastSendBTIntent(directionJSONObj.toString());

            // updating robot information (coordinates and direction) to be displayed
            TextView xAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_x_coord);
            TextView yAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_y_coord);
            String newDirText_x = "X: " + String.valueOf(gridMap.getXCoord());
            String newDirText_y = "Y: " + String.valueOf(gridMap.getYCoord());
            TextView directionAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robotDirText);
            directionAxisTextView.setText("DIR: "+ String.valueOf(gridMap.getRobotFacingDir()));
            xAxisTextView.setText(newDirText_x);
            yAxisTextView.setText(newDirText_y);
        }catch (Exception e){
            Log.e(TAG, "sendDirCmdIntent: An error occured while sending direction command intent");
            e.printStackTrace();
        }
    }

    private void sendControlCmdIntent(String control){
        try{
            JSONObject ctrlJSONObj = new JSONObject();
            ctrlJSONObj.put("cat","control");
            ctrlJSONObj.put("value",control);

            broadcastSendBTIntent(ctrlJSONObj.toString());
        }catch (Exception e){
            Log.e(TAG, "sendControlCmdIntent: An error occured while sending control command intent");
            e.printStackTrace();
        }
    }

    // Send an Intent with an action named "sendBTMessage"
    // the Intent sent should be received by the sendBluetoothReceiver (ReceiverActivity)
    private void broadcastSendBTIntent(String msg){
        Intent sendBTIntent = new Intent("sendBTMessage");
        sendBTIntent.putExtra("msg",msg);
        LocalBroadcastManager.getInstance(getContext()).sendBroadcast(sendBTIntent);
    }

    private void showShortToast(String msg) {
        Toast.makeText(getActivity(), msg, Toast.LENGTH_SHORT).show();
    }

    private class ObstacleListItem {
        int obsNo;
        int x;
        int y;
        String facing;

        public ObstacleListItem(int obsNo,int x, int y, String facing){
            this.obsNo = obsNo;
            this.x=x;
            this.y=y;
            this.facing=facing;
        }
    }

    private class ObstaclesListViewAdapter extends ArrayAdapter<ObstacleListItem>{
        private List<ObstacleListItem> items;

        public ObstaclesListViewAdapter(@NonNull Context context, int resource, @NonNull List<ObstacleListItem> objects) {
            super(context, resource, objects);
            items=objects;
        }

        public void updateList(List<ObstacleListItem> list) {
            this.items = list;
            this.notifyDataSetChanged();
        }

        @NonNull
        @Override
        public View getView(int position, @Nullable View convertView, @NonNull ViewGroup parent) {
            if (convertView == null) {
                convertView = LayoutInflater.from(getContext()).inflate(R.layout.obstacle_list_item_layout, parent, false);
            }
            ObstacleListItem item = items.get(position);
            TextView obsNoTxt = (TextView) convertView.findViewById(R.id.obstacleListItemNumTextView);
            TextView xPosTxt = (TextView) convertView.findViewById(R.id.obstacleListItemXCoordTextView);
            TextView yPosTxt = (TextView) convertView.findViewById(R.id.obstacleListItemYCoordTextView);
            TextView facingTxt = (TextView) convertView.findViewById(R.id.obstacleListItemDirTextView);

            obsNoTxt.setText("#"+item.obsNo);
            xPosTxt.setText(Integer.toString(item.x));
            yPosTxt.setText(Integer.toString(item.y));
            facingTxt.setText(item.facing);

            return convertView;
        }
    }
}