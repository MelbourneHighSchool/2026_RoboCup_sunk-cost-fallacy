#include <iostream>
#include <array>
#include <cmath>
#include <algorithm>
#include <random>
#include <functional>
#include <thread>
#include <mutex>
using namespace std; //yeah bad practice whatever

const float fieldW = 1820, fieldh = 2430, fieldRW = 910, fieldRH = 1215;

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
bernoulli_distribution disBool(0.9);
auto rnd = bind(dis01f, gen);
auto rndBool = bind(disBool, gen);
array<int,8> targetDists;
float targetAngle;
array<float,8> dists;
float lowestErr, error, temperature;
float bestX, bestY, bestAngle;
float guessX, guessY, guessAngle;
char action;

void localiseLoop(){
    guessX = 0; guessY = 0; 
    bestX = guessX; bestY = guessY;
    inlock.lock();
    inlock.unlock();
    guessAngle = targetAngle;
    bestAngle = targetAngle;
    //cin >> angle;
    estDistances(&guessX,&guessY,&guessAngle,dists.begin());
    error = 0;
    for (int i = 0; i < 8; i++){
        error += abs(dists[i] - targetDists[i]);
    }
    lowestErr = error;
    while (true){
        temperature = error / 12;
        if (rndBool()){
            guessX = clamp(guessX + (rnd() * temperature) - (temperature / 2),-fieldRW + 105, fieldRW - 105);
            guessY = clamp(guessY + (rnd() * temperature) - (temperature / 2),-fieldRH + 105, fieldRH - 105);;
        } else {
            guessAngle += (rnd()-0.5) / 50;
            
            if (guessAngle < -M_PI) {guessAngle += (2*M_PI);}
            if (guessAngle > M_PI) {guessAngle -= (2*M_PI);}
        }
        estDistances(&guessX, &guessY, &guessAngle, dists.begin());
        inlock.lock();
        error = min(min(guessAngle - targetAngle + (float)(2*M_PI),targetAngle - guessAngle + (float)(2*M_PI)),abs(targetAngle-guessAngle));
        for (int j = 0; j < 8; j++){
            error += abs(max(targetDists[j]-dists[j],-100.0f));
        }
        inlock.unlock();
        if (error < lowestErr){
            lowestErr = error;
            outlock.lock();
            bestX = guessX;
            bestY = guessY;
            bestAngle = guessAngle;
            outlock.unlock();
        } else{
            guessX = bestX;
            guessY = bestY;
            guessAngle = bestAngle;
        }
    }
}
array<float,8> mdists;
float merr;
int main(){
    //handles io
    cin.tie(nullptr);
    ios_base::sync_with_stdio(false);
    inlock.lock();
    thread t(localiseLoop);
    t.detach(); //if it breaks its because of this line i think probably
    while (true){
        cin >> action;
        if (action == 'i'){
            for (int i = 0; i < 8; i++){
                cin >> targetDists[i];
            }
            cin >> targetAngle;
            bestAngle = targetAngle;
            estDistances(&bestX,&bestY,&bestAngle,mdists.begin());
            merr = min(min(bestAngle - targetAngle + (float)(2*M_PI),targetAngle - bestAngle + (float)(2*M_PI)),abs(bestAngle-guessAngle));
            for (int i = 0; i < 8; i++){
                merr += abs(max(targetDists[i]-mdists[i],-100.0f));
            }
            lowestErr = merr;
            break;
        } else if (action == 'o'){
            cout << -9999 << "\n" << -9999 << "\n" << -9999 <<"\n";
            cout.flush();
        } else if (action == 'e'){
            return 0;
        } else {
            cerr << "invalid action\n";
        }
    }
    inlock.unlock();
    while (true){
        cin >> action;
        if (action == 'i'){
            inlock.lock();
            for (int i = 0; i < 8; i++){
                cin >> targetDists[i];
            }
            cin >> targetAngle;
            inlock.unlock();
            bestAngle = targetAngle;
            outlock.lock();
            estDistances(&bestX,&bestY,&bestAngle,mdists.begin());
            outlock.unlock();
            merr = min(min(bestAngle - targetAngle + (float)(2*M_PI),targetAngle - bestAngle + (float)(2*M_PI)),abs(bestAngle-guessAngle));
            for (int i = 0; i < 8; i++){
                merr += abs(max(targetDists[i]-mdists[i],-100.0f));
            }
            lowestErr = merr;
        } else if (action == 'o'){
            outlock.lock();
            cout << bestX << "\n" << bestY << "\n" << bestAngle <<"\n";
            cout.flush();
            outlock.unlock();
        } else if (action == 'e'){
            return 0;
        } else {
            cerr << "invalid action\n";
        }
    }
}