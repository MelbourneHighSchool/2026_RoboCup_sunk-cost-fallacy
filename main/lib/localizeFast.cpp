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

const float fieldW = 1820, fieldh = 2430, fieldRW = 910, fieldRH = 1215, fieldGoalRH = 0;


array<float,8> rayX;
array<float,8> rayY;

mutex inlock;
mutex outlock;

/// @brief Simulates distances recorded on the tofs.
/// @param px x position of the robot
/// @param py y position of the robot
/// @param a bearing of the robot (0 forward, + clockwise)
/// @return array of simulated tof distances
void estDistances(float* px, float* py, float* a, array<float,8>::iterator start){
    
    for (int i = 0; i < 8; i++){
        rayX[i] = sin(i*M_PI_4 + *a);
        rayY[i] = cos(i*M_PI_4 + *a);
    }
    auto cur = start;
    for (int i = 0; i < 8; i++){
        *cur = min(
            abs(
                (fieldRW - copysign(*px, *px*rayX[i]) )
                /rayX[i]
            ),
            abs(
                (fieldRH - copysign(*py, *py*rayY[i]) )
                /rayY[i]
            )
        );
        cur++;
    }
}

random_device rd;
mt19937 gen(rd());
uniform_real_distribution<float> dis01f(0.0, 1.0);
normal_distribution<float> disNormal(0,1);
bernoulli_distribution disBool(1);
auto rnd = bind(dis01f, gen);
auto rndBool = bind(disBool, gen);
auto rndNorm = bind(disNormal, gen);
array<int,8> targetDists;
float targetAngle;
array<float,8> dists;
float lowestErr, error, temperature;
float bestX, bestY, bestAngle;
float guessX, guessY, guessAngle;
char action;

/// @brief Function for daemon localization thread. do you spell localization with s or z? idk.
void localiseLoop(){
    //initialise stuff
    guessX = 230; guessY = -310; 
    bestX = guessX; bestY = guessY;
    inlock.lock();
    inlock.unlock();
    guessAngle = targetAngle;
    bestAngle = targetAngle;
    //cin >> angle;
    estDistances(&guessX,&guessY,&guessAngle,dists.begin());
    error = 0;
    lowestErr = error;
    while (true){
        //modify guess position randomly
        temperature = error/30;
        guessAngle = targetAngle;
        guessX = clamp(guessX + (rndNorm() * temperature),-fieldRW + 105, fieldRW - 105);
        guessY = clamp(guessY + (rndNorm() * temperature),-fieldRH + 105, fieldRH - 105);
        //simulate sensor measurements for this position
        estDistances(&guessX, &guessY, &guessAngle, dists.begin());
        //calculate difference between actual measurements and these measurements
        inlock.lock();
        error = 0;//min(min(guessAngle - targetAngle + (float)(2*M_PI),targetAngle - guessAngle + (float)(2*M_PI)),abs(targetAngle-guessAngle));
        for (int j = 0; j < 8; j++){
            error += abs(max(targetDists[j]-dists[j],-100.0f));
        }
        inlock.unlock();
        //if this position has lower error then stored position, update stored position
        if (error < lowestErr){
            lowestErr = error;
            outlock.lock();
            bestX = guessX;
            bestY = guessY;
            //bestAngle = guessAngle;
            outlock.unlock();
        } else{
            guessX = bestX;
            guessY = bestY;
        }
    }
}
array<float,8> mdists;
array<queue<float>,8> prevDists;
array<float,8> avgPrevDists;
const int numSmoothingMeasurements = 8;
float merr;
int tIdx;
float tValue;
int main(){
    //main loop handles io. boring, don't want to annotate
    targetAngle = 0;
    for (int i = 0; i < 8; i++){
        targetDists[i] = 0;
        avgPrevDists[i] = 1000 * numSmoothingMeasurements;
        for (int _ = 0; _ < numSmoothingMeasurements; _++){
            prevDists[i].push(1000);
        }
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
                if (abs(((numSmoothingMeasurements*tValue)/avgPrevDists[tIdx]) - 1) <0.35){
                    targetDists[tIdx] = tValue;
                }
                avgPrevDists[tIdx] += tValue;
                avgPrevDists[tIdx] -= prevDists[tIdx].front();
                prevDists[tIdx].push(tValue);
                prevDists[tIdx].pop();
            } 
            inlock.unlock();
            bestAngle = targetAngle;
            outlock.lock();
            estDistances(&bestX,&bestY,&bestAngle,mdists.begin());
            outlock.unlock();
            merr = 0;//min(min(bestAngle - targetAngle + (float)(2*M_PI),targetAngle - bestAngle + (float)(2*M_PI)),abs(bestAngle-guessAngle));
            for (int i = 0; i < 8; i++){
                merr += abs(max(targetDists[i]-mdists[i],-100.0f));
            }
            lowestErr = merr;
        } else if (action == 'o'){
            outlock.lock();
            cout << bestX << "\n" << bestY << "\n" << bestAngle <<"\n";
            inlock.lock();
            for (int i = 0; i < 8; i++){
                cout << targetDists[i] << " ";
            }
            cout << "\n" << lowestErr << "\n";
            inlock.unlock();
            cout.flush();
            outlock.unlock();
        } else if (action == 'e'){
            return 0;
        } else {
            cerr << "bad input\n";
        }
    }
}