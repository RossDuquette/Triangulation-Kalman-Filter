#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <math.h>

#include "viewer.h"

Viewer::Viewer(std::vector<Vehicle>& vehicles, std::vector<Beacon>& beacons) :
    vehicles_(vehicles),
    beacons_(beacons),
    width_(800),
    height_(800)
{
    int argc = 0;
    char** argv;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(width_, height_);
    glutCreateWindow("Particle Filter");
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    update();
}

void Viewer::update()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    draw_vehicles();
    draw_beacons();
    glutSwapBuffers();
}

void Viewer::draw_obj(float x, float y)
{
    glRectf(x * 20 / width_, y * 20 / height_,
            (x + 1) * 20 / width_, (y + 1) * 20 / height_);
}

void Viewer::draw_circle(float x, float y, float r)
{
    x = ((x / width_) + 1) * width_ / 2;
    y = ((y / height_) + 1) * height_ / 2;

    glColor3f(0.0f,  1.0f, 0.0f);
    glBegin(GL_POLYGON);
    for (int i = 0; i < 360; i++) {
        float theta = i * M_PI / 180.0;
        glVertex2f(x + r * cos(theta), y + r * sin(theta));
    }
    glEnd();
}

void Viewer::draw_vehicles()
{
    for (int i = 0; i < vehicles_.size(); i++) {
        glColor3f(0.0f, 0.0f, 0.0f);
        Pose2D& pose = vehicles_[i].get_pose();
        draw_obj(pose.x, pose.y);
    }
}

void Viewer::draw_beacons()
{
    for (int i = 0; i < beacons_.size(); i++) {
        glColor3f(0.0f, 0.0f, 1.0f);
        Position2D& pos = beacons_[i].get_position();
        draw_obj(pos.x, pos.y);
    }
}

void Viewer::draw_distances()
{
    for (int i = 0; i < vehicles_.size(); i++) {
        for (int j = 0; j < beacons_.size(); j++) {
            Position2D& pos = beacons_[j].get_position();
            draw_circle(pos.x, pos.y, distance(pos, vehicles_[i].get_pose()));
        }
    }
}


