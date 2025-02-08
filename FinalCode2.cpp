#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <queue>
#include <set>
#include <map>
#include <tuple>
#include <stdexcept>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

 struct position{

    int x;
    int y;



 };

 // Global overload for equality.
bool operator==(const position &a, const position &b) {
    return a.x == b.x && a.y == b.y;
}

// Global overload for less-than (required by std::set and std::map).
bool operator<(const position &a, const position &b) {
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}


 struct AstarNode{

    position pos;
    double gcost;
    double fcost;

    AstarNode(const position& p, double g, double f) : pos(p), gcost(g), fcost(f) {}

    bool operator>(const AstarNode &other) const {
        return fcost > other.fcost;
    }


 };


bool isValidPosition(int row, int col, vector<vector<char>>& grid) {
        return row >= 0 && row < grid.size() && col >= 0 && col < grid[0].size();
    }



bool isFrontier(int row, int col, vector<vector<char>>& grid){

    if (isValidPosition(row,col,grid) && grid[row][col] != '.') return false;

    const vector<position> directions = {{1,0},{-1,0},{0,1},{0,-1}};

    for (const auto &[dr,dc]:directions){

        int nextRow = row + dr;
        int nextCol = col + dc;

        if (isValidPosition(nextRow,nextCol,grid) && grid[nextRow][nextCol]=='?'){
            return true;
        }


    }
    return false;
 };

 

set<position> bfs(vector<vector<char>>& grid, position start){

    set<position> reached;
    set<position> frontier;
    queue<position> search;

    reached.insert(start);

    search.push(start);

    const vector<position> directions = {{1,0},{-1,0},{0,1},{0,-1}};
     //const std::vector<position> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1},{-1,-1},{-1,1},{1,-1},{1,1}};

    while(!search.empty()){

        position current = search.front();

        search.pop();

        for (const auto &[dr,dc]:directions){

            int nextRow = current.x + dr;
            int nextCol = current.y + dc;

            if(isFrontier(nextRow,nextCol,grid)){
                frontier.insert({nextRow,nextCol});
            }

            if(isValidPosition(nextRow,nextCol,grid) && reached.find({nextRow, nextCol}) == reached.end()){
                reached.insert({nextRow,nextCol});

                if(grid[nextRow][nextCol]=='.' || grid[nextRow][nextCol]=='T'){
                    search.push({nextRow,nextCol});
                }

                 if(grid[nextRow][nextCol]=='C'){
             


                }
            }
        }
    }

    return frontier;
};

auto heuristic(const position& current, const position& goal){

    return std::abs(goal.x-current.x)+std::abs(goal.y-current.y);

};


vector<position> Astar(vector<vector<char>>& grid, position start, position goal){

    set<position> closed;

    map<position, position> came_from;

    std::priority_queue<AstarNode, std::vector<AstarNode>, std::greater<AstarNode>> open;

    map<position, double> cost_so_far;

    const vector<position> directions = {{1,0},{-1,0},{0,1},{0,-1}};
     //const std::vector<position> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1},{-1,-1},{-1,1},{1,-1},{1,1}};

    AstarNode startnode(start,0,heuristic(start,goal));

    open.emplace(startnode);
    cost_so_far[start] = 0;

    while(!open.empty()){
        AstarNode currentnode = open.top();
        open.pop();

        if(currentnode.pos == goal){

            vector<position> path;

            for (position p = goal; p != start ; p = came_from[p]) {
                path.push_back(p);
            }

            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }

        if(grid[currentnode.pos.x][currentnode.pos.y] == '?'){

            vector<position> path;

            for (position p = goal; p != start ; p = came_from[p]) {
                path.push_back(p);
            }

            path.push_back(start);
            reverse(path.begin(), path.end());
            return path;
        }


        closed.insert(currentnode.pos);
        
        for (const auto &[dr,dc]:directions){

            position neighbour = {currentnode.pos.x+dr, currentnode.pos.y+dc};

            if (!isValidPosition(neighbour.x,neighbour.y,grid) || grid[neighbour.x][neighbour.y] !='.' && grid[neighbour.x][neighbour.y] !='C' && grid[neighbour.x][neighbour.y] !='T' || closed.count(neighbour)){
                continue;
            };

            double tentativeGCost = cost_so_far[currentnode.pos] + 1 ; // Include wall penalty

            if (!cost_so_far.count(neighbour) || tentativeGCost < cost_so_far[neighbour]) {
                came_from[neighbour] = currentnode.pos;
                cost_so_far[neighbour] = tentativeGCost;
                double fCost = tentativeGCost + heuristic(neighbour, goal);

                open.emplace(neighbour, tentativeGCost, fCost); // Use constructor here
            }

        }
    }
    return {}; // Return an empty path if no path is found

};

// Assumes that 'position' is already defined and has public int members x and y.

// Function that computes the Manhattan distance between two positions.
int manhattanDistance(const position &a, const position &b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

// Finds the closest point in 'points' to the 'target' using Manhattan distance.
// Throws a runtime_error if 'points' is empty.
position findClosest(const std::set<position>& points, const position& target) {
    if (points.empty()) {
        throw std::runtime_error("The set of positions is empty!");
    }

    auto it = points.begin();
    position closest = *it;
    int minDistance = manhattanDistance(closest, target);

    for (const auto &p : points) {
        int currentDistance = manhattanDistance(p, target);
        if (currentDistance < minDistance) {
            minDistance = currentDistance;
            closest = p;
        }
    }
    return closest;
}

std::vector<std::string> getDirectionsFromPath(const std::vector<position>& path) {
    std::vector<std::string> directions;
    
    // If the path has fewer than 2 positions, there is no move to compute.
   /* if (path.size() < 2) {
     directions.push_back("WAIT");
    return directions;
    }*/

    // Iterate over the path, starting from the second position.
    for (std::size_t i = 1; i < path.size(); ++i) {
        // Calculate differences between consecutive positions.
        int dx = path[i].x - path[i - 1].x;
        int dy = path[i].y - path[i - 1].y;
        
        // Determine direction based on the differences.
        if (dx == 1 && dy == 0) {
            directions.push_back("DOWN");
        } else if (dx == -1 && dy == 0) {
            directions.push_back("UP");
        } else if (dx == 0 && dy == 1) {
            directions.push_back("RIGHT");
        } else if (dx == 0 && dy == -1) {
            directions.push_back("LEFT");
        } else {
            // In case the step isn't exactly one cell in a cardinal direction,
            // you can either throw an error, ignore it, or store a placeholder.
            directions.push_back("INVALID");
        }
    }
    
    return directions;
}

position controlcheck(vector<vector<char>>& grid){

    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[0].size();j++){
            if(grid[i][j]=='C'){

                position controlPos ={i,j};
                return controlPos;
            }
            
        }
    }
    position pos = {0,0};
    return pos;
}


position telecheck(vector<vector<char>>& grid){

    for(int i=0;i<grid.size();i++){
        for(int j=0;j<grid[0].size();j++){
            if(grid[i][j]=='T'){
                position controlPos ={i,j};
                return controlPos;
            }
            
        }
    }
}

int main()
{
    int r; // number of rows.
    int c; // number of columns.
    int a; // number of rounds between the time the alarm countdown is activated and the time the alarm goes off.
    cin >> r >> c >> a; cin.ignore();

    int state = 0;

    position controlPos = {0,0};

    bool controlFound=false;


    // game loop
    while (1) {

        vector<vector<char>> map;   //grid creation https://github.com/texus/codingame/blob/master/SingePlayer/Hard/The%20Labyrinth.cpp

        int kr; // row where Rick is located.
        int kc; // column where Rick is located.
        cin >> kr >> kc; cin.ignore();


        for (int i = 0; i < r; i++) {
            string row; // C of the characters in '#.TC?' (i.e. one line of the ASCII maze).
            cin >> row; cin.ignore();

            map.push_back({});
            for (char c : row)
                map.back().push_back(c);

        }
        position rickpos = {kr,kc};



        auto telepos = telecheck(map);
        controlPos = controlcheck(map);

        if(controlPos.x != 0 && controlPos.y !=0 && !controlFound){

            state=1;
            controlFound=true;
  
        }

        if(rickpos==controlPos){
          state=2;
        }



        cerr<<controlPos.x<<" "<<controlPos.y<<endl;
        cerr<<state<<endl;

        switch(state){
            

            case 0: {

             auto frontiers = bfs(map,rickpos);

             auto movepos = findClosest(frontiers,rickpos);

             auto path = Astar(map,rickpos,movepos);

             auto moves = getDirectionsFromPath(path);

             cout<<moves[0]<<endl;

             break;
             
            }

            case 1:

            {

            auto pathToControl = Astar(map,rickpos,controlPos);
            
            auto movesToControl = getDirectionsFromPath(pathToControl);

            if(!movesToControl.empty()){
                cout<<movesToControl[0]<<endl;

            }

            else {

            auto frontiers = bfs(map,rickpos);
            auto movepos = findClosest(frontiers,rickpos);
            auto path = Astar(map,rickpos,movepos);
            auto moves = getDirectionsFromPath(path);
            cout<<moves[0]<<endl;

            }    
    
            break;
            
            }

            case 2:
            {

            auto pathToTele = Astar(map, rickpos, telepos);
            auto movesToTele = getDirectionsFromPath(pathToTele);

            cout<<movesToTele[0]<<endl;

            break;
            }

        }

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;

    }
}