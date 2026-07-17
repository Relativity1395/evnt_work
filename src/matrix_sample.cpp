#include <libcaer/libcaer.h>
#include <libcaer/devices/dvxplorer.h>
#include <libcaer/events/polarity.h>

// #include <opencv2/opencv.hpp>

#include <signal.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstdint>
#include <cmath>
#include <vector>
#include <iostream>

int matrix_mult(int A, int B){

}

int main(void){
const int ROWS = 3;
const int COLS = 4;

int A[ROWS][COLS] = {
 {-1, 0, 1, -1},
 {-1,-1,-1,0},
 {1,1,1,1}
    
};

int B[COLS][ROWS] = {
    {-1, -1, 1},
    {0, -1, 1},
    {1,-1,1},
    {-1,0,1}
};

int M[ROWS][COLS] = {{0}};
int N[ROWS][ROWS] = {{0}};
int W[COLS]= {1, 0, 1,0};

for (int i = 0; i < ROWS; i++){
    for (int j = 0; j < COLS; j++){
        
        M[i][j]= A[i][j]*W[j];
       
        
            
        // std::cout<<j<<std::endl;;
        
        // std::cout<< M[i][j] << std::endl;;
        // std::cout<<"\n"<<std::endl;;
        
    }
}



for (int i = 0; i < ROWS; i++){
    for (int j = 0; j < COLS; j++){
        N[i][j] += M[i][j] * B[j][i];
        
        if (j == COLS - 1){
            std::cout<< N[i][j] << std::endl;;
        }
        
    }
    
}

return 0;
}