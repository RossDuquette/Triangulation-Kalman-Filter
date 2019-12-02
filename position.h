#ifndef POSITION_H_
#define POSITION_H_

class Position2D
{
    public:
        Position2D(float x_, float y_);
        float x;
        float y;
};

class Pose2D : public Position2D
{
    public:
        Pose2D(float x_, float y_, float theta_);
        float theta;
};

float distance(const Position2D& pos1, const Position2D& pos2);

#endif
