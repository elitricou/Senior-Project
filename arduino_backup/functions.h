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
    Coordinate calculateCoordinates(int phi, int theta,float heightOfPost);
    //generatePath(int array [][], seed); //incorporate motion sensor into the pathing algorithm
    void kinematics(int length, int array[][50], int p1_phi, int p1_theta, int p2_phi, int p2_theta, float heightOfPost);
    void pathing(int length, int array[][50], int seed);
};

#endif