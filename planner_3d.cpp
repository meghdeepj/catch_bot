/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <time.h>
#include <mex.h>
#include <iostream>
#include <vector>
#include <stack>
#include <set>
#include <unordered_map>

using namespace std;

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

typedef pair<int, vector<int>> listPair;

stack<pair<int,int>> Path;
bool have_path = false;


struct cell{

    vector<int> parent;
    int f, g, h;

    cell()
        : parent({-1, -1, -1})
        , f(-1)
        , g(-1)
        , h(-1)
    {
    }
};

struct ArrayHasher {
    std::size_t operator()(const std::array<int, 3>& a) const {
        std::size_t h = 0;
        for (auto e : a) {
            h ^= std::hash<int>{}(e)  + 0x9e3779b9 + (h << 6) + (h >> 2); 
        }
        return h;
    }   
};

bool isValid(int x, int y, int x_size, int y_size, double* map, int collision_thresh){
    if(((int)map[GETMAPINDEX(x,y,x_size,y_size)] >= 0) && ((int)map[GETMAPINDEX(x,y,x_size,y_size)] < collision_thresh)){
        return true;
    }
    return false;
}

// vector<int> seach_2d(){

//     int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
//     int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};

//     int goalposeX = (int) target_traj[curr_time+(int)(delta*dist_to_object)];
//     int goalposeY = (int) target_traj[curr_time+target_steps+(int)(delta*dist_to_object)];

//     // int goalposeX = (int) target_traj[target_steps-1];
//     // int goalposeY = (int) target_traj[target_steps-1+target_steps];

//     // int goalposeX = targetposeX;
//     // int goalposeY = targetposeY;


//     if(dist_to_object<20 || (int)(*(&target_traj + 1) - target_traj)==0){
//         int goalposeX = targetposeX;
//         int goalposeY = targetposeY;
//     }

//     mexPrintf("\n targetpose is %d,%d", targetposeX, targetposeY);
//     mexPrintf("\n goalpose is %d,%d", goalposeX, goalposeY);

//     vector<vector<bool>> closed(x_size, vector<bool> (y_size, false));
    
//     int i, j;

//     vector<vector<cell>> grid(x_size, vector<cell>(y_size));

//     for (i = 0; i < x_size; i++) {
//         for (j = 0; j < y_size; j++) {
//             grid[i][j].f = INT_MAX;
//             grid[i][j].g = INT_MAX;
//             grid[i][j].h = INT_MAX;
//             grid[i][j].parent = make_pair(-1, -1);
//         }
//     }

//     i=robotposeX, j=robotposeY;

//     grid[i][j].f = 0;
//     grid[i][j].g = 0;
//     grid[i][j].h = 0;
//     grid[i][j].parent = make_pair(i, j);

//     set<listPair> open;
//     open.insert(make_pair(0, make_pair(i, j)));
    
//     int newx, newy;
//     vector<int> new_pose={robotposeX, robotposeY};
//     bool found_path = false;
//     if(robotposeX!=goalposeX && robotposeY!=goalposeY){
//         while (!open.empty()) {
//             listPair curr = *open.begin(); // remove s with the smallest f(s) from OPEN;
//             open.erase(open.begin()); // remove s from OPEN
//             i = curr.second.first;
//             j = curr.second.second;
//             closed[i][j] = true; //insert s into CLOSED
//             // mexPrintf("\n child %d, %d", i, j);
//             int gNew, hNew, fNew;
//             for(int dir = 0; dir < NUMOFDIRS; dir++)
//             {
//                 newx = i + dX[dir];
//                 newy = j + dY[dir];
//                 // mexPrintf("\n child %d, %d", newx, newy);

//                 if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)  //if new pose is within the map
//                 {   
//                     // mexPrintf("\n opensize %d", open.size());      
//                     if (newx==goalposeX && newy==goalposeY)  //if new pose is the goal pose
//                     {
//                         // mexPrintf("\n 3");
//                         grid[newx][newy].parent.first = i;
//                         grid[newx][newy].parent.second = j;
//                         new_pose= getPath(grid, goalposeX, goalposeY);
//                         found_path=true;
//                     }
//                     else if(closed[newx][newy]==false && isValid(newx,newy,x_size,y_size,map,collision_thresh)) // if new pose is not in CLOSED and is valid
//                     {
//                         gNew = grid[i][j].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
//                         hNew = (int)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
//                         fNew = gNew + hNew;
//                         // mexPrintf("\n 4");
//                         if (grid[newx][newy].g == INT_MAX || grid[newx][newy].g > gNew) //if g(s')>g(s)+c(s,s')
//                         {
//                             open.insert(make_pair(fNew, make_pair(newx, newy))); // insert s' in OPEN
//                             // mexPrintf("\n 5");
//                             grid[newx][newy].f = fNew;  
//                             grid[newx][newy].g = gNew;  // update g(s')
//                             grid[newx][newy].h = hNew;
//                             grid[newx][newy].parent = make_pair(i, j);
//                         }
//                     }
//                 }
//             }
//             if(found_path) break;   
//         }
//     }
// }


vector<int> getPath(unordered_map<array<int,3> , cell, ArrayHasher >& grid, int goalposeX, int goalposeY, int goalT)
{
    int row = goalposeX;
    int col = goalposeY;
    int t = goalT;
 
    while (!(grid[{row,col,t}].parent[0] == row
             && grid[{row,col,t}].parent[1] == col && grid[{row,col,t}].parent[2] == t))
    {
        Path.push(make_pair(row, col));
        int temp_row = grid[{row,col,t}].parent[0];
        int temp_col = grid[{row,col,t}].parent[1];
        int temp_time = grid[{row,col,t}].parent[2];
        row = temp_row;
        col = temp_col;
        t = temp_time;
    }
 
    pair<int, int> p = Path.top();
    return {p.first, p.second};
}

static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{    
    if(curr_time==0 && have_path==true && Path.size()>0){
        have_path=false;
        Path.pop();
    }
    if(have_path)
    {
        mexPrintf("\nPath empty: %d", Path.size());
        if(Path.size()>1){
            Path.pop();
            pair<int, int> p = Path.top();
            action_ptr[0] = p.first;
            action_ptr[1] = p.second;
        }else{
            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;
        }
        mexPrintf("\n Next pose: %.2f,%.2f", action_ptr[0], action_ptr[1]);
        return;
    }
    // 8-connected grid
    int dX[NUMOFDIRS+1] = {0, -1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS+1] = {0, -1,  0,  1, -1,  1, -1, 0, 1};
    int dT = 1;

    vector<int> new_pose={robotposeX, robotposeY};
    clock_t tStart = clock();
    double delta=0.1, epsilon=50;
    double dist_to_object = sqrt(((robotposeX-targetposeX)*(robotposeX-targetposeX) + (robotposeY-targetposeY)*(robotposeY-targetposeY)));
    
    int goalposeX = (int) target_traj[target_steps-1];
    int goalposeY = (int) target_traj[target_steps-1+target_steps];

    mexPrintf("\n targetpose is %d,%d", targetposeX, targetposeY);
    mexPrintf("\n goalpose is %d,%d", goalposeX, goalposeY);

    // vector<vector<vector<cell>>> grid(x_size, vector<vector<cell>> (y_size, vector<cell>));  // 3d grid to store the graph
    unordered_map<array<int,3> , cell, ArrayHasher >  grid;
    // unordered_map<vector<int>,cell> grid;
    unordered_map<array<int,3> , bool, ArrayHasher >  closed;
    // unordered_map<vector<int>,bool> closed;   // closed list
    set<listPair> open;

    int i, j, k;

    i=robotposeX, j=robotposeY, k=0;
    cell c = {};
    c.parent = vector<int>{i, j, k};
    c.g = 0;
    c.f = 0;
    c.h = 0;
    grid[{i,j,k}] = c;
    
    open.insert(make_pair(0, vector<int> {i,j,k}));
    int newx, newy, newt;
    bool found_path = false;
    int num_expanded = 0;
    while (!open.empty()) {
        // if(newt+time_elapsed>= target_steps){
        //     new_pose = search_2d(map,x_size,y_size,goalposeX, goalposeY, target_steps-1);
        // }
        listPair curr = *open.begin(); // remove s with the smallest f(s) from OPEN;
        open.erase(open.begin()); // remove s from OPEN
        
        i = curr.second[0];
        j = curr.second[1];
        k = curr.second[2];

        closed[{i,j,k}] = true; //insert s into CLOSED
        int gNew, hNew, fNew;
        num_expanded++;
        mexPrintf("\n num_expanded: %d", num_expanded);
        for(int dir = 0; dir < NUMOFDIRS; dir++)
        {
            newx = i + dX[dir];
            newy = j + dY[dir];
            newt = k + dT;
            // mexPrintf("\n child %d, %d", newx, newy);
            int time_elapsed = 5 + (int)((clock() - tStart)/CLOCKS_PER_SEC);
            int target_x = (int) target_traj[curr_time+time_elapsed+newt]; 
            int target_y = (int) target_traj[curr_time+time_elapsed+newt+target_steps]; 
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size && newt+time_elapsed<=target_steps)  //if new pose is within the map
            {                      
                if (newx==target_x && newy==target_y)  //if new pose is the goal pose at that time
                {
                    mexPrintf("\n newt: %d, time_elapsed: %d", newt, time_elapsed);
                    mexPrintf("\n target_x: %d, target_y: %d", target_x, target_y);
                    grid[{newx,newy,newt}].parent = {i,j,k};
                    new_pose= getPath(grid, target_x, target_y, newt);
                    // new_pose= getPath(grid, goalposeX, goalposeY, newt);
                    found_path=true;
                    have_path=true;
                }
                else if( (closed.find({newx,newy,newt}) == closed.end() || closed[{newx,newy,newt}]==false) && isValid(newx,newy,x_size,y_size,map,collision_thresh)) // if new pose is not in CLOSED and is valid
                {
                    gNew = grid[{i,j,k}].g + (int)map[GETMAPINDEX(newx,newy,x_size,y_size)];
                    // hNew = (int)epsilon*(sqrt(((newx-target_x)*(newx-target_x) + (newy-target_y)*(newy-target_y) + (newt)*(newt))));
                    hNew = (int) epsilon*(sqrt(2)*MIN(abs(newx-target_x),abs(newy-target_y))+(MAX(abs(newx-target_x),abs(newy-target_y))-MIN(abs(newx-target_x),abs(newy-target_y))));

                    fNew = gNew + hNew;
                    // mexPrintf("\n 4");
                    if(grid.find({newx,newy,newt})==grid.end())
                    {  // if node is not in grid construct graph
                            cell c = {};
                            c.parent = vector<int>{-1,-1,-1};
                            c.g = INT_MAX;
                            c.h = INT_MAX;
                            c.f = INT_MAX;
                            grid[{newx,newy,newt}] = c;
                            
                    }  
                    if (grid[{newx,newy,newt}].g == INT_MAX || grid[{newx,newy,newt}].g > gNew) //if g(s')>g(s)+c(s,s')
                    {
                        open.insert(make_pair(fNew,  vector<int> {newx,newy,newt})); // insert s' in OPEN
                        // mexPrintf("\n 5");
                        grid[{newx,newy,newt}].f = fNew;  
                        grid[{newx,newy,newt}].g = gNew;  // update g(s')
                        grid[{newx,newy,newt}].h = hNew;
                        grid[{newx,newy,newt}].parent =  vector<int> {i,j,k};
                    }
                }
            }
        }
        if(found_path) break;   
    }

    // :::::::::::::::::::::: planner :::::::::::::::::::::::::::::::::::::::::::::::::
    mexPrintf("\n found path %d", found_path);
    mexPrintf("\n robot: %d %d", robotposeX, robotposeY);
    mexPrintf("\n next goal is %d,%d \n", new_pose[0], new_pose[1]);

    robotposeX = new_pose[0];
    robotposeY = new_pose[1];
    action_ptr[0] = robotposeX;
    action_ptr[1] = robotposeY;
    mexPrintf("Time taken: %.2fs\n", (double)(clock() - tStart)/CLOCKS_PER_SEC);
    // mexPrintf("object: %d %d;\n", (int) target_traj[curr_time],(int) target_traj[curr_time+target_steps]);
    // mexPrintf("dist_to_object: %d ;\n", dist_to_object);
    
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}