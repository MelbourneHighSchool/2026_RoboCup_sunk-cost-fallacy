#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>
#include <random>
#include <functional>
#include <thread>
#include <mutex>
#include <queue>
using namespace std; //yeah bad practice whatever

const int 
        fieldW = 1820, 
        fieldh = 2430, 
        fieldRW = 910, 
        fieldRH = 1215, 
        fieldGoalRH = 0,
        minWallDist = 110,
        maxRayDist = 2900; 
const int tofRadius = 50;

array<float,8> rayX;
array<float,8> rayY;

mutex inlock;
mutex outlock;

/// @brief Simulates distances recorded on the tofs.
/// @param px x position of the robot
/// @param py y position of the robot
/// @param a bearing of the robot (0 forward, + clockwise)
/// @return array of simulated tof distances
const array<array<int, 3>, 6> VERTICAL_WALLS = {{
    {{910,  1215, -1215}},   // right field boundary
    {{-910, 1215, -1215}},   // left field boundary
    {{225,  1215,   940}},   // top goal, right post
    {{-225, 1215,   940}},   // top goal, left post
    {{225, -1215,  -940}},   // bottom goal, right post
    {{-225,-1215,  -940}},   // bottom goal, left post
}};
const array<array<int, 3>, 4> HORIZONTAL_WALLS = {{
    {{1215,   910, -910}},   // top field boundary
    {{-1215,  910, -910}},   // bottom field boundary
    {{989,    225, -225}},   // top goal back wall
    {{-989,   225, -225}},   // bottom goal back wall
}};
const float small = 1e-9; 
void estDistances(const int* px, const int* py, const float* a, array<float,8>::iterator cur){
    rayX[0] = sin(*a);          rayY[0] = cos(*a);
    rayX[1] = sin(*a + M_PI_4); rayY[1] = cos(*a + M_PI_4);
    rayX[2] = rayY[0]; rayY[2] = -rayX[0];
    rayX[3] = rayY[1]; rayY[3] = -rayX[1]; 

    for (int i = 0; i < 4; i++){
        rayX[i + 4] = -rayX[i];
        rayY[i + 4] = -rayY[i];
    }
    //noahn's code
    for (int i = 0; i < 8; i++){
        *cur = 2900;
        // vertical walls (y = C)
        if (abs(rayX[i]) > small)
        {

            for (int j = 0; j < 6; j++){
                const float s = (VERTICAL_WALLS[j][0]- *px) / rayX[i];
                if (small < s and s < (*cur)) {
                    const float hit = *py + s * rayY[i];
                    const float t = (hit - VERTICAL_WALLS[j][1]) / (VERTICAL_WALLS[j][2] - VERTICAL_WALLS[j][1]);
                    if (-small <= t and t <= 1 + small){
                        (*cur) = s;
                    }
                }
            }
        }
        // horizontal walls (y = C)
        if (abs(rayY[i]) > small) {
            for (int j = 0; j < 4; j++){
                const float s = (HORIZONTAL_WALLS[j][0]- *py) / rayY[i];
                if (small < s and s < (*cur)) {
                    const float hit = *px + s * rayX[i];
                    const float t = (hit - HORIZONTAL_WALLS[j][1]) / (HORIZONTAL_WALLS[j][2] - HORIZONTAL_WALLS[j][1]);
                    if (-small <= t and t <= 1 + small){
                        (*cur) = s;
                    }
                }
            }
        }
        cur++;
    }
}

random_device rd;
mt19937 gen(rd());
uniform_real_distribution<float> dis01f(0.0, 1.0);
uniform_real_distribution<float> dism11f(-1.0,1.0);
bernoulli_distribution disBool(0.5);
auto rnd = bind(dis01f, gen);
auto rndBool = bind(disBool, gen);
auto rndGuess = bind(dism11f, gen);
array<float,8> targetDists;
float targetAngle;
array<float,8> dists;
array<bool, 8> badToF;
float lowestErr, error, guessRadius;
int bestX, bestY, guessX, guessY;
float bestAngle, guessAngle;
char action;

void calcTotalError(float* out, array<float,8>::iterator ptrGuess) {
    *out = 0;
    auto ptrTarget = targetDists.begin();
    auto ptrBad = badToF.begin();
    //float div = 0;
    for (unsigned int i = 0; i < 8; i++){
        if (not *ptrBad) {
            *out +=  abs(*ptrGuess - *ptrTarget); //* (*ptrTarget);
            //div += (*ptrTarget);
        } 
        ptrGuess++; ptrTarget++; ptrBad++;
    }
    //*out /= div;
}

/// @brief Function for daemon localization thread. do you spell localization with s or z? idk.
void localiseLoop(){
    //initialise stuff
    guessX = 0; guessY = 0; 
    bestX = guessX; bestY = guessY;
    inlock.lock();
    inlock.unlock();
    guessAngle = targetAngle;
    bestAngle = targetAngle;
    //cin >> angle;
    estDistances(&guessX,&guessY,&guessAngle,dists.begin());
    calcTotalError(&lowestErr, dists.begin());
    
    while (true){
        //modify guess position randomly
        guessRadius = error/12;
        guessAngle = targetAngle;
        if (rndBool()){
            guessX = clamp(guessX + (int)round(rndGuess() * guessRadius),-fieldRW + minWallDist, fieldRW - minWallDist);
        } else{
            guessY = clamp(guessY + (int)round(rndGuess() * guessRadius),-fieldRH + minWallDist, fieldRH - minWallDist);
        }
        //simulate sensor measurements for this position
        estDistances(&guessX, &guessY, &guessAngle, dists.begin());
        //calculate difference between actual measurements and these measurements
        inlock.lock();
        calcTotalError(&error, dists.begin());
        
        inlock.unlock();
        //if this position has lower error then stored position, update stored position
        outlock.lock();
        if (error < lowestErr){
            lowestErr = error;
            bestX = guessX;
            bestY = guessY;
            //bestAngle = guessAngle;
        } else{
            guessX = bestX;
            guessY = bestY;
        }
        outlock.unlock();
    }
}
array<float,8> mdists;
float merr;
int tIdx;
float tValue;
int main(){
    //main loop handles io. boring, don't want to annotate
    targetAngle = 0;
    for (int i = 0; i < 8; i++){
        targetDists[i] = 0;
        badToF[i] = false;
    }
    cin.tie(nullptr);
    ios_base::sync_with_stdio(false);
    thread t(localiseLoop);
    t.detach(); 
    while (true){
        cin >> action;
        if (action == 'i'){
            cin >> tIdx >> tValue;
            if (cin.bad()){
                cerr << "bad input\n";
                cin.clear();
                continue;
            }
            inlock.lock();
            if (tIdx == 8){
                targetAngle = tValue;
            } else {
                if (tValue < (200 - tofRadius) or (tValue + targetDists[(tIdx + 4) % 8] + tofRadius > maxRayDist)){
                    badToF[tIdx] = true;
                    // prevent bad tof from erroneously labeling the opposite ToF as bad
                    targetDists[tIdx] = minWallDist; 
                } else{
                    badToF[tIdx] = false;
                    targetDists[tIdx] = tValue + tofRadius;
                }
            } 
            inlock.unlock();
            bestAngle = targetAngle;
            outlock.lock();
            estDistances(&bestX,&bestY,&bestAngle,mdists.begin());
            calcTotalError(&merr, mdists.begin());
            lowestErr = merr;
            outlock.unlock();
        } else if (action == 'o'){
            outlock.lock();
            cout << bestX << "\n" << bestY << "\n";
            outlock.unlock();
            cout.flush();
        } else if (action == 'd'){
            for (int i = 0; i < 8; i++){
                cout << targetDists[i] << " ";
            }
            cout << "\n";
            for (int i = 0; i < 8; i++){
                cout << badToF[i] << " ";
            }
            cout << "\n" << lowestErr << "\n";
            cout.flush();
        } else if (action == 'e'){
            return 0;
        } else {
            cerr << "bad input\n";
        }
    }
}