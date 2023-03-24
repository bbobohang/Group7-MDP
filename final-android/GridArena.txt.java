package com.example.androidapp;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.Nullable;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;

public class GridArena extends View {
    public GridArena(Context c) {
        super(c);
        initMap();
    }

    //set everything below as false default
    private static boolean startCoordStatus = false;
    private static boolean setObstacleStatus = false;
    private static boolean setObstacleDirection = false;
    private static boolean canDrawRobot = false;
    private boolean mapDrawn = false;
    private static boolean obstacleSelectedStatus = false;
    private static Direction robotFrontFacing = Direction.NONE;
    private static int[] currCoord = new int[]{-1, -1};
    private static ArrayList<int[]> obstacleCoord = new ArrayList<>();
    private static final String TAG = "GridArena";
    private static final int COL = 20;
    private static final int ROW = 20;
    private static float cellSize;
    private static Cell[][] gridCells;
    private static int[] selectedObstacleCoord = new int[2];
    private static ArrayList<Cell> oCellArr = new ArrayList<Cell>();
    private Paint gridNoText =  new Paint();
    private Paint obstacleColor = new Paint();
    private Paint robotColor = new Paint();
    private Paint endColor = new Paint();
    private Paint startColor = new Paint();
    private Paint unexploredColor = new Paint();
    private Paint imgLine = new Paint();
    private Paint comfirmImgLine = new Paint();
    private Paint blackPaint = new Paint();
    private Paint whitePaint = new Paint();

    // 0 = None 1 = Up, 2 = Down, 3 = Left, 4 = Right
    int switchDirection = -1;
    String[] directionList = new String[]{"NONE", "UP", "DOWN", "LEFT", "RIGHT"};
    private static int[] obstacleNoArray = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

    public GridArena(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        initMap();
        comfirmImgLine.setStyle(Paint.Style.STROKE);
        comfirmImgLine.setColor(Color.YELLOW);
        comfirmImgLine.setStrokeWidth(5);
        imgLine.setStyle(Paint.Style.STROKE);
        imgLine.setColor(Color.YELLOW);
        imgLine.setStrokeWidth(2);

        obstacleColor.setColor(Color.BLACK);
        robotColor.setColor(Color.CYAN);
        endColor.setColor(Color.GRAY);
        startColor.setColor(Color.CYAN);
        unexploredColor.setColor(Color.LTGRAY);
        gridNoText.setColor(Color.WHITE);
        gridNoText.setTextSize(15);
        gridNoText.setFakeBoldText(true);
        blackPaint.setStyle(Paint.Style.FILL_AND_STROKE);
        whitePaint.setColor(Color.WHITE);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (!mapDrawn) {
            drawCell();
            resetGrid();
            mapDrawn = true;
        }
        drawIndivGridCell(canvas);
        drawGridLines(canvas);
        drawGridNumber(canvas);

        if (canDrawRobot) drawRobot(canvas);
    }

    private void drawIndivGridCell(Canvas canvas) {
        for (int x = 0; x < COL + 2; x++) {
            for (int y = 0; y < ROW + 2; y++) {
                Cell cell = gridCells[x][y];

                // drawing gridCells here
                canvas.drawRect(cell.startX, cell.startY, cell.endX, cell.endY, cell.paint);
                // drawing obstacle number and direction
                if (cell.type == Type.OBSTACLE) {
                    //Number for obstacle
                    if (cell.targetID == null) {
                        canvas.drawText(Integer.toString(cell.obstacleNo), cell.startX + (cellSize / 3.2f), cell.startY + (cellSize / 1.5f), whitePaint);
                    } else {
                        Paint targetPaint = new Paint();
                        targetPaint.setTextSize(20);
                        targetPaint.setColor(Color.GREEN);
                        targetPaint.setTextAlign(Paint.Align.CENTER);
                        canvas.drawText(cell.targetID, (cell.startX + cell.endX) / 2, cell.endY + (cell.startY - cell.endY) / 4, targetPaint);
                    }

                    // Draw where the obstacle is facing
                    if (cell.obstacleFacing != null || cell.obstacleFacing == Direction.NONE) {
                        switch (cell.obstacleFacing) {
                            case LEFT:
                                canvas.drawRect(cell.startX + 1, cell.startY + 2, cell.endX - (cellSize / 1.1f), cell.endY, imgLine);
                                break;
                            case RIGHT:
                                canvas.drawRect(cell.startX + (cellSize / 1f) - 2, cell.startY, cell.endX - 1, cell.endY, imgLine);
                                break;
                            case UP:
                                canvas.drawRect(cell.startX + 2, cell.startY + 1, cell.endX, cell.endY - (cellSize / 1.1f), imgLine);
                                break;
                            case DOWN:
                                canvas.drawRect(cell.startX + 2, cell.startY + (cellSize / 1f) - 2, cell.endX, cell.endY - 1, imgLine);
                                break;
                        }
                    }
                }
                if (cell.type == Type.ROBOT && getRobotFacingDir() != null) {
                    int[] cellIndexes = convertMapCoordToCellsIndexes(currCoord[0], currCoord[1]);
                    int indexX = cellIndexes[0];
                    int indexY = cellIndexes[1];
                    switch (robotFrontFacing) {
                        case RIGHT:
                            canvas.drawLine(gridCells[indexX - 1][indexY - 1].startX, gridCells[indexX - 1][indexY - 1].startY, gridCells[indexX + 1][indexY].endX, gridCells[indexX + 1][indexY - 1].endY + (gridCells[indexX + 1][indexY].endY - gridCells[indexX + 1][indexY - 1].endY) / 2, blackPaint);
                            canvas.drawLine(gridCells[indexX + 1][indexY].endX, gridCells[indexX + 1][indexY - 1].endY + (gridCells[indexX + 1][indexY].endY - gridCells[indexX + 1][indexY - 1].endY) / 2, gridCells[indexX - 1][indexY + 1].startX, gridCells[indexX - 1][indexY + 1].endY, blackPaint);
                            break;
                        case LEFT:
                            canvas.drawLine(gridCells[indexX + 1][indexY - 1].endX, gridCells[indexX + 1][indexY - 1].startY, gridCells[indexX - 1][indexY].startX, gridCells[indexX - 1][indexY - 1].endY + (gridCells[indexX - 1][indexY].endY - gridCells[indexX - 1][indexY - 1].endY) / 2, blackPaint);
                            canvas.drawLine(gridCells[indexX - 1][indexY].startX, gridCells[indexX - 1][indexY - 1].endY + (gridCells[indexX - 1][indexY].endY - gridCells[indexX - 1][indexY - 1].endY) / 2, gridCells[indexX + 1][indexY + 1].endX, gridCells[indexX + 1][indexY + 1].endY, blackPaint);
                            break;
                        case UP:
                            //left drawn line
                            canvas.drawLine(gridCells[indexX - 1][indexY + 1].startX, gridCells[indexX - 1][indexY + 1].endY, (gridCells[indexX][indexY - 1].startX + gridCells[indexX][indexY - 1].endX) / 2, gridCells[indexX][indexY - 1].startY, blackPaint);
                            //right drawn line
                            canvas.drawLine((gridCells[indexX][indexY - 1].startX + gridCells[indexX][indexY - 1].endX) / 2, gridCells[indexX][indexY - 1].startY, gridCells[indexX + 1][indexY + 1].endX, gridCells[indexX + 1][indexY + 1].endY, blackPaint);
                            break;
                        case DOWN:
                            canvas.drawLine(gridCells[indexX - 1][indexY - 1].startX, gridCells[indexX - 1][indexY - 1].startY, (gridCells[indexX][indexY + 1].startX + gridCells[indexX][indexY + 1].endX) / 2, gridCells[indexX][indexY + 1].endY, blackPaint);
                            canvas.drawLine((gridCells[indexX][indexY + 1].startX + gridCells[indexX][indexY + 1].endX) / 2, gridCells[indexX][indexY + 1].endY, gridCells[indexX + 1][indexY - 1].endX, gridCells[indexX + 1][indexY - 1].startY, blackPaint);
                            break;
                        default:
                            Toast.makeText(this.getContext(), "Error with drawing robot (unknown direction)", Toast.LENGTH_LONG).show();
                            break;
                    }
                }
            }
        }
    }

    private Cell getGridCellCoords(int x, int y) {

        return gridCells[x + 1][COL - y];
    }

    public void updateObsImageID(int obstacleNo, String targetID) {
        // searching obstacle number with same id
        for (int x = 1; x <= COL; x++)
            for (int y = 1; y <= ROW; y++)
                if (gridCells[x][y].obstacleNo == obstacleNo) {
                    gridCells[x][y].targetID = targetID;
                }
        this.invalidate();
    }

    private void drawGridLines(Canvas canvas) {
        for (int y = 0; y <= COL; y++) {
            Cell start = gridCells[1][y];
            Cell end = gridCells[COL][y];
            canvas.drawLine(start.startX, start.endY, end.endX, end.endY, blackPaint);
        }
        for (int x = 1; x <= COL + 1; x++) {
            Cell start = gridCells[x][1];
            Cell end = gridCells[x][ROW];
            canvas.drawLine(start.startX, start.startY, end.startX, end.endY, blackPaint);
        }
    }

    private void drawGridNumber(Canvas canvas) {
        // X-axis
        for (int x = 1; x <= COL; x++) {
            Cell cell = gridCells[x][COL + 1];
            String num = "" + (x - 1);
            if (x > 9)
                canvas.drawText(num, cell.startX + (cellSize / 5), cell.startY + (cellSize / 2), gridNoText);
            else
                canvas.drawText(num, cell.startX + (cellSize / 3), cell.startY + (cellSize / 2), gridNoText);
        }
        // Y-axis
        for (int y = 1; y <= ROW; y++) {
            Cell cell = gridCells[0][y];
            int adjustedY = ROW - y;
            String num = "" + adjustedY;
            if (adjustedY > 9)
                canvas.drawText(num, cell.startX + (cellSize / 3), cell.startY + (cellSize / 1.5f), gridNoText);
            else
                canvas.drawText(num, cell.startX + (cellSize / 2), cell.startY + (cellSize / 1.5f), gridNoText);
        }
    }

    private void drawRobot(Canvas canvas) {
        if(currCoord[0] == -1 || currCoord[1] == -1){
            return;
        }

        int[] cellIndexes = convertMapCoordToCellsIndexes(currCoord[0],currCoord[1]);
        int indexX = cellIndexes[0];
        int indexY = cellIndexes[1];

        switch (robotFrontFacing) {
            case RIGHT:
                canvas.drawLine(gridCells[indexX - 1][indexY - 1].startX, gridCells[indexX - 1][indexY - 1].startY, gridCells[indexX + 1][indexY].endX, gridCells[indexX + 1][indexY - 1].endY + (gridCells[indexX + 1][indexY].endY - gridCells[indexX + 1][indexY - 1].endY) / 2, blackPaint);
                canvas.drawLine(gridCells[indexX + 1][indexY].endX, gridCells[indexX + 1][indexY - 1].endY + (gridCells[indexX + 1][indexY].endY - gridCells[indexX + 1][indexY - 1].endY) / 2, gridCells[indexX - 1][indexY + 1].startX, gridCells[indexX - 1][indexY + 1].endY, blackPaint);
                break;
            case LEFT:
                canvas.drawLine(gridCells[indexX + 1][indexY - 1].endX, gridCells[indexX + 1][indexY - 1].startY, gridCells[indexX - 1][indexY].startX, gridCells[indexX - 1][indexY - 1].endY + (gridCells[indexX - 1][indexY].endY - gridCells[indexX - 1][indexY - 1].endY) / 2, blackPaint);
                canvas.drawLine(gridCells[indexX - 1][indexY].startX, gridCells[indexX - 1][indexY - 1].endY + (gridCells[indexX - 1][indexY].endY - gridCells[indexX - 1][indexY - 1].endY) / 2, gridCells[indexX + 1][indexY + 1].endX, gridCells[indexX + 1][indexY + 1].endY, blackPaint);
                break;
            case UP:
                canvas.drawLine(gridCells[indexX - 1][indexY + 1].startX, gridCells[indexX - 1][indexY + 1].endY, (gridCells[indexX][indexY - 1].startX + gridCells[indexX][indexY - 1].endX) / 2, gridCells[indexX][indexY - 1].startY, blackPaint);
                canvas.drawLine((gridCells[indexX][indexY - 1].startX + gridCells[indexX][indexY - 1].endX) / 2, gridCells[indexX][indexY - 1].startY, gridCells[indexX + 1][indexY + 1].endX, gridCells[indexX + 1][indexY + 1].endY, blackPaint);
                break;
            case DOWN:
                canvas.drawLine(gridCells[indexX - 1][indexY - 1].startX, gridCells[indexX - 1][indexY - 1].startY, (gridCells[indexX][indexY + 1].startX + gridCells[indexX][indexY + 1].endX) / 2, gridCells[indexX][indexY + 1].endY, blackPaint);
                canvas.drawLine((gridCells[indexX][indexY + 1].startX + gridCells[indexX][indexY + 1].endX) / 2, gridCells[indexX][indexY + 1].endY, gridCells[indexX + 1][indexY - 1].endX, gridCells[indexX + 1][indexY - 1].startY, blackPaint);
                break;
            default:
                Toast.makeText(this.getContext(), "Unknown direction", Toast.LENGTH_LONG).show();
                break;
        }
    }

    private void drawCell() {
        gridCells = new Cell[COL + 2][ROW + 2];
        cellSize = getWidth() / (COL + 2);

        for (int x = 0; x < COL + 2; x++) {
            for (int y = 0; y < ROW + 2; y++) {
                float startX = x * cellSize;
                float startY = y * cellSize;
                gridCells[x][y] = new Cell(startX, startY, startX + cellSize, startY + cellSize, Type.UNEXPLORED);
            }
        }
        //Setting borders
        for (int x = 0; x < COL + 2; x++) {
            gridCells[x][0].setType(Type.BORDER);
            gridCells[x][ROW + 1].setType(Type.BORDER);
        }
        for (int y = 0; y < ROW + 2; y++) {
            gridCells[0][y].setType(Type.BORDER);
            gridCells[COL + 1][y].setType(Type.BORDER);
        }
    }

    public void updateCurCoord(int mapX, int mapY, Direction direction) {
        if(currCoord[0] != -1 && currCoord[1] != -1){
            int[] oldCoordIndexes = convertMapCoordToCellsIndexes(currCoord[0],currCoord[1]);
            int oldCoordXindex = oldCoordIndexes[0];
            int oldCoordYindex = oldCoordIndexes[1];
            for (int x = oldCoordXindex - 1; x <= oldCoordXindex + 1; x++) {
                for (int y = oldCoordYindex - 1; y <= oldCoordYindex + 1; y++) {
                    if (gridCells[x][y].type == Type.ROBOT) {
                        gridCells[x][y].setType(Type.UNEXPLORED);
                    }
                }
            }
        }
        //Updating location
        robotFrontFacing = direction;
        currCoord[0] = mapX;
        currCoord[1] = mapY;
        int[] newCoordIndexes = convertMapCoordToCellsIndexes(mapX,mapY);
        int newCoordXIndex = newCoordIndexes[0];
        int newCoordYIndex = newCoordIndexes[1];
        for (int x = newCoordXIndex - 1; x <= newCoordXIndex + 1; x++) {
            for (int y = newCoordYIndex - 1; y <= newCoordYIndex + 1; y++) {
                if (gridCells[x][y].type != Type.OBSTACLE && gridCells[x][y].type != Type.BORDER) {
                    gridCells[x][y].setType(Type.ROBOT);
                }
            }
        }
        invalidate();
    }

    public void setRobotFacingDir(Direction direction) {
        robotFrontFacing = direction;
        TextView directionAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robotDirText);
        directionAxisTextView.setText("DIR: " + direction.toString());
        this.invalidate();
    }

    private void updateRobotInfoTextView(int col, int row, Direction direction) {
        TextView xAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_x_coord);
        TextView yAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_y_coord);
        TextView directionAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robotDirText);

        String newDirText_x = "X: " + String.valueOf(col - 1);
        String newDirText_y = "Y: " + String.valueOf(row - 1);

        xAxisTextView.setText(newDirText_x);
        yAxisTextView.setText(newDirText_y);
        directionAxisTextView.setText("DIR: " + direction.toString());
    }

    protected void setObstacleCoord(int mapX, int mapY) {
        // Checking if obstacle is set before
        if (getGridCellCoords(mapX, mapY).type == Type.OBSTACLE) {
            return;
        }
        int[] obstacleCoord = new int[]{mapX, mapY};
        GridArena.obstacleCoord.add(obstacleCoord);

        Cell newObsCell = getGridCellCoords(mapX, mapY);
        newObsCell.setType(Type.OBSTACLE);
        // Giving obstacle number
        for (int i = 0; i < obstacleNoArray.length; i++) {
            if (obstacleNoArray[i] != -1) {
                if (newObsCell.obstacleNo == -1) {
                    newObsCell.obstacleNo = obstacleNoArray[i];
                    obstacleNoArray[i] = -1;
                    break;
                }
            }
        }
        this.invalidate();
        updateObsListView();
    }

    protected void removeObstacle(int mapX, int mapY){
        Cell removeObstacleCell = getCellAtMapCoord(mapX, mapY);
        if(removeObstacleCell.type != Type.OBSTACLE){
            return;
        }
        // Return available obstacle number
        int oldObstacleNo = removeObstacleCell.obstacleNo;
        obstacleNoArray[oldObstacleNo-1] = oldObstacleNo;
        // Reset obstacle cell
        removeObstacleCell.obstacleNo=-1;
        removeObstacleCell.targetID=null;
        removeObstacleCell.obstacleFacing = Direction.NONE;
        removeObstacleCell.setType(Type.UNEXPLORED);
        // Remove from arraylist
        for(int i = 0; i <obstacleCoord.size(); i++){
            int[] coord = obstacleCoord.get(i);
            if(coord[0] == mapX && coord[1]==mapY){
                obstacleCoord.remove(i);
                break;
            }
        }
        this.invalidate();
        updateObsListView();
    }

    private class Cell {
        float startX, startY, endX, endY;
        Paint paint;
        Type type;
        int id = -1;
        Direction obstacleFacing = Direction.NONE;
        String targetID = null;
        int obstacleNo = -1;

        private Cell(float startX, float startY, float endX, float endY, Type type) {
            this.startX = startX;
            this.startY = startY;
            this.endX = endX;
            this.endY = endY;
            setType(type);
        }

        public void setType(Type type) {
            this.type = type;
            switch (type) {
                case ROBOT:
                    this.paint = robotColor;
                    break;
                case BORDER:
                    this.paint = endColor;
                    break;
                case OBSTACLE:
                    this.paint = obstacleColor;
                    break;
                case UNEXPLORED:
                    this.paint = unexploredColor;
                    break;
                default:
                    break;
            }
        }

        public void setObstacleFacing(Direction obstacleFacing) {
            this.obstacleFacing = obstacleFacing;
        }

        public void setId(int id) {
            this.id = id;
        }

        public int getId() {
            return this.id;
        }
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int mapX = (int) (event.getX() / cellSize) - 1;
        int mapY = ROW - ((int) (event.getY() / cellSize));

        Cell selectedCell = null;
        if ((mapX >= 0 && mapY >= 0 && mapX <= COL - 1 && mapY <= ROW - 1)) {
            selectedCell = getCellAtMapCoord(mapX, mapY);
        }

        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            if (startCoordStatus && selectedCell != null) {
                updateCurCoord(mapX,mapY,Direction.UP);
                canDrawRobot = true;
                invalidate();
                startPlaceRobot();
                return true;
            }
            if (setObstacleStatus && selectedCell != null) {
                Log.i(TAG, "onTouchEvent: Adding Obstacle at X: " + mapX + " Y: " + mapY);
                this.setObstacleCoord(mapX, mapY);
                return true;
            }
            if (setObstacleDirection && selectedCell != null) startSetFacing(selectedCell);

            if (!obstacleSelectedStatus && selectedCell != null) {
                for (int i = 0; i < obstacleCoord.size(); i++)
                    if (obstacleCoord.get(i)[0] == mapX && obstacleCoord.get(i)[1] == mapY) {
                        selectedObstacleCoord[0] = mapX;
                        selectedObstacleCoord[1] = mapY;
                        obstacleSelectedStatus = true;
                        return true;
                    }
            }
        } else if(event.getAction() == MotionEvent.ACTION_UP){
            if (obstacleSelectedStatus) {
                obstacleSelectedStatus = false;
                return true;
            }
        } else if(event.getAction() == MotionEvent.ACTION_MOVE){
            if (obstacleSelectedStatus) {
                boolean occupied = false;
                for (int i = 0; i < obstacleCoord.size(); i++) {
                    if (obstacleCoord.get(i)[0] == mapX && obstacleCoord.get(i)[1] == mapY) {
                        occupied = true;
                    }
                }
                if (occupied == false) {
                    Cell oldObstacleCell = getCellAtMapCoord(selectedObstacleCoord[0], selectedObstacleCoord[1]);
                    // Caching old obstacle direction
                    Direction oldObstacleDir = oldObstacleCell.obstacleFacing;
                    String oldTargetID = oldObstacleCell.targetID;
                    removeObstacle(selectedObstacleCoord[0], selectedObstacleCoord[1]);

                    //If selection is within the grid; move to new position
                    if (mapX < 20 && mapY < 20 && mapX > 0 && mapY > 0) {
                        // Update selectedObstacleCoord;
                        selectedObstacleCoord[0] = mapX;
                        selectedObstacleCoord[1] = mapY;

                        setObstacleCoord(mapX,mapY);
                        selectedCell.obstacleFacing = oldObstacleDir;
                        selectedCell.targetID = oldTargetID;
                    }
                    this.invalidate();
                    return true;
                }

            }
        }
        return false;
    }

    private void startSetFacing(Cell selectedCell) {
        boolean isSetRobot = (selectedCell.type == Type.ROBOT);
        AlertDialog.Builder mBuilder = new AlertDialog.Builder(getContext());
        mBuilder.setTitle("Select Direction");
        mBuilder.setSingleChoiceItems(directionList, switchDirection, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                switchDirection = i;
            }
        });
        mBuilder.setNeutralButton("OK", new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialogInterface, int i) {
                Direction selectedDirection = Direction.NONE;
                switch (switchDirection) {
                    case 0:
                        selectedDirection = Direction.NONE;
                        break;
                    case 1:
                        selectedDirection = Direction.UP;
                        break;
                    case 2:
                        selectedDirection = Direction.DOWN;
                        break;
                    case 3:
                        selectedDirection = Direction.LEFT;
                        break;
                    case 4:
                        selectedDirection = Direction.RIGHT;
                        break;
                }

                if(isSetRobot && selectedDirection == Direction.NONE) {
                    setRobotFacingDir(Direction.UP);
                } else if(isSetRobot) {
                    setRobotFacingDir(selectedDirection);
                } else {
                    selectedCell.setObstacleFacing(selectedDirection);
                    updateObsListView();
                }
                invalidate();
                dialogInterface.dismiss();
            }
        });

        if(selectedCell.type == Type.ROBOT || selectedCell.type == Type.OBSTACLE) {
            AlertDialog dialog = mBuilder.create();
            dialog.show();
        }
    }

    public void startPlaceRobot() {
        if(!startCoordStatus) return;
        setStartCoordStatus(false);
        // updating robot X and Y coordinates
        TextView xAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_x_coord);
        TextView yAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robot_y_coord);
        String newDirText_x = "X: " + String.valueOf(currCoord[0]);
        String newDirText_y = "Y: " + String.valueOf(currCoord[1]);
        TextView directionAxisTextView = ((Activity) this.getContext()).findViewById(R.id.robotDirText);
        directionAxisTextView.setText("DIR: "+ String.valueOf(getRobotFacingDir()));
        xAxisTextView.setText(newDirText_x);
        yAxisTextView.setText(newDirText_y);
        

        //Re-enable other buttons
        Button placeRobotBtn = ((Activity) this.getContext()).findViewById(R.id.placeRobotBtn);
        Button setObstacleBtn  = ((Activity) this.getContext()).findViewById(R.id.setObstacleBtn);
        Button setFacingBtn = ((Activity) this.getContext()).findViewById(R.id.setFacingBtn);
        Button resetArenaBtn = ((Activity) this.getContext()).findViewById(R.id.resetArenaBtn);
        Button startFastestBtn = ((Activity) this.getContext()).findViewById(R.id.startFastestBtn);
        Button startImageRecBtn = ((Activity) this.getContext()).findViewById(R.id.startImageRecBtn);

        setObstacleBtn.setEnabled(true);
        setFacingBtn.setEnabled(true);
        resetArenaBtn.setEnabled(true);
        startFastestBtn.setEnabled(true);
        startImageRecBtn.setEnabled(true);
        placeRobotBtn.setText("Place Robot");
    }

    public void resetGrid() {
        TextView robotStatusTextView = ((Activity) this.getContext()).findViewById(R.id.robotStatusText);
        updateRobotInfoTextView(1, 1, Direction.NONE);
        robotStatusTextView.setText("Not Available");
        currCoord = new int[]{-1, -1};
        robotFrontFacing = Direction.NONE;
        obstacleCoord = new ArrayList<>();
        canDrawRobot = false;
        oCellArr = new ArrayList<>();
        obstacleNoArray = new int[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
        mapDrawn = false;
        updateObsListView();
        this.invalidate();
    }

    private int dirToInt(Direction direction) {
        switch (direction) {
            case UP:
                return 0;
            case DOWN:
                return 4;
            case RIGHT:
                return 2;
            case LEFT:
                return 6;
            default:
                return -1;
        }
    }

    public void sendUpdatedObstacleInformation() {
        try {
            JSONArray obstaclesList = new JSONArray();

            for (int i = 0; i < obstacleCoord.size(); i++) {
                JSONObject obstacle = new JSONObject();
                int obstacleX = obstacleCoord.get(i)[0];
                int obstacleY = obstacleCoord.get(i)[1];
                Cell obstacleCell = getCellAtMapCoord(obstacleX, obstacleY);
                obstacle.put("x", obstacleX);
                obstacle.put("y", obstacleY);
                obstacle.put("id", obstacleCell.obstacleNo);
                obstacle.put("d", dirToInt(obstacleCell.obstacleFacing));
                obstaclesList.put(obstacle);
            }

            // obstacle output format: 3 key value pairs
            JSONObject msgJSON = new JSONObject();
            msgJSON.put("cat", "obstacles");
            msgJSON.put("obstacles", obstaclesList);
            msgJSON.put("mode", "0");

            // printing out new obstacle output
            Log.d(TAG, "Obstacle Information Output: " + msgJSON);

            Intent upDirectionIntent = new Intent("sendBTMessage");
            upDirectionIntent.putExtra("msg", msgJSON.toString());
            LocalBroadcastManager.getInstance(getContext()).sendBroadcast(upDirectionIntent);
        } catch (Exception ex) {
            Log.e(TAG, "sendUpdatedObstacleInformation: An error occurred while sending obstacle information to device");
            ex.printStackTrace();
        }
    }

    private void updateObsListView() {
        try{
            JSONArray obstacleInfo = new JSONArray();
            for(int[] obstacleCoord : obstacleCoord){
                JSONObject obstalceObj = new JSONObject();
                Cell cell = getGridCellCoords(obstacleCoord[0],obstacleCoord[1]);
                obstalceObj.put("no",cell.obstacleNo);
                obstalceObj.put("x",obstacleCoord[0]);
                obstalceObj.put("y",obstacleCoord[1]);
                obstalceObj.put("facing",cell.obstacleFacing.toString());

                obstacleInfo.put(obstalceObj);
            }

            Intent obstacleListIntent = new Intent("newObstacleList");
            obstacleListIntent.putExtra("msg", obstacleInfo.toString());
            LocalBroadcastManager.getInstance(getContext()).sendBroadcast(obstacleListIntent);
        }catch (Exception e){
            Log.e(TAG, "updateFrontEndListView: Error adding obstacle to JSON");
        }
    }

    public void removeAllTargetIDs(){
        try{
            for (int i = 0; i < obstacleCoord.size(); i++) {
                int obstacleX = obstacleCoord.get(i)[0];
                int obstacleY = obstacleCoord.get(i)[1];
                Cell obstacleCell = getCellAtMapCoord(obstacleX, obstacleY);
                obstacleCell.targetID = null;
            }
            invalidate();
        }catch (Exception ex) {
            Log.e(TAG, "removeAllObstacleIDs: An error occurred while removing confirmed target IDs");
        }
    }
    private Cell getCellAtMapCoord(int x, int y) {
        return gridCells[x + 1][ROW - y];
    }

    private int[] convertMapCoordToCellsIndexes(int mapX, int mapY){
        int[] convertedCoords = {mapX+1,ROW-mapY};
        return convertedCoords;
    }

    private void initMap() {
        setWillNotDraw(false);
    }

    private enum Type {
        UNEXPLORED,
        OBSTACLE,
        ROBOT,
        BORDER
    }

    public enum Direction{
        NONE,
        UP,
        DOWN,
        LEFT,
        RIGHT
    }

    public void setSetObstacleDirection(boolean status) {
        setObstacleDirection = status;
    }

    public void setSetObstacleStatus(boolean status) {
        setObstacleStatus = status;
    }

    public void setStartCoordStatus(boolean status) {
        startCoordStatus = status;
    }

    public int getXCoord() {

        return currCoord[0];
    }

    public int getYCoord() {
        return currCoord[1];
    }

    public Direction getRobotFacingDir() {

        return robotFrontFacing;
    }
}
