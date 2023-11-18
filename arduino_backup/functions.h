#ifndef functions_h
#define functions_h


class functions {
public:
    functions();
    
    struct Coordinate {
        float x;
        float y;
    };

    float find_X(int phi, int theta, float heightOfPost);
    float find_Y(int phi, int theta, float heightOfPost);
    float find_Phi(float X, float Y);
    float find_Theta(float X, float Y, float heightOfPost);
    double calc_distance(int row, int col, float heightOfPost);
    Coordinate calculateCoordinates(int phi, int theta,float heightOfPost);
    void writeArrEeprom(int length, int array[], int row);
    void readArrEeprom(int length, int array[], int row);
    void kinematics(int length, int array[], int p1_phi, int p1_theta, int p2_phi, int p2_theta, float heightOfPost, int row);
   
};

#endif