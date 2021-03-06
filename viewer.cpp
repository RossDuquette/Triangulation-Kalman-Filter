#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <math.h>

#include "viewer.h"

Viewer::Viewer(Vehicle& vehicle, std::vector<Beacon>& beacons,
               ParticleFilter& particle, KalmanFilter& kalman, float size_m) :
    vehicle_(vehicle),
    beacons_(beacons),
    particle_(particle),
    kalman_(kalman),
    size_m_(size_m),
    size_pix_(800)
{
    int argc = 0;
    char** argv;
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(size_pix_, size_pix_);
    glutCreateWindow("Particle Filter");
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    update();
}

void Viewer::update()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    draw_vehicle();
    draw_beacons();
    draw_distances();
    draw_particles();
    draw_estimate();
    glutSwapBuffers();
}

float Viewer::m_to_pix(float m)
{
    return m / size_m_;
}

void Viewer::draw_square(float x, float y, float size)
{
    float bl_x = m_to_pix(x - size / 2.0);
    float bl_y = m_to_pix(y - size / 2.0);
    float tr_x = m_to_pix(x + size / 2.0);
    float tr_y = m_to_pix(y + size / 2.0);

    glRectf(bl_x, bl_y, tr_x, tr_y);
}

void Viewer::draw_circle(float x, float y, float r)
{
    x = m_to_pix(x);
    y = m_to_pix(y);
    r = m_to_pix(r);

    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < 360; i++) {
        float theta = i * M_PI / 180.0;
        glVertex2f(x + r * cos(theta), y + r * sin(theta));
    }
    glEnd();
}

void Viewer::draw_vehicle()
{
    const float VEHICLE_SIZE_M = 1;
    set_colour(BLACK);
    Pose2D pose = vehicle_.get_pose();
    draw_square(pose.x, pose.y, VEHICLE_SIZE_M);
}

void Viewer::draw_beacons()
{
    const float BEACON_SIZE_M = 0.4;
    set_colour(BLUE);
    for (int i = 0; i < beacons_.size(); i++) {
        Position2D pos = beacons_[i].get_position();
        draw_square(pos.x, pos.y, BEACON_SIZE_M);
    }
}

void Viewer::draw_distances()
{
    set_colour(RED);
    for (int i = 0; i < beacons_.size(); i++) {
        Position2D pos = beacons_[i].get_position();
        draw_circle(pos.x, pos.y, beacons_[i].get_distance());
    }
}

void Viewer::draw_particles()
{
    const float PARTICLE_SIZE_M = 0.3;
    set_colour(BLUE);
    std::vector<Position2D>& particles = particle_.get_particles();
    for (int i = 0; i < particles.size(); i++) {
        draw_square(particles[i].x, particles[i].y, PARTICLE_SIZE_M);
    }
}

void Viewer::draw_estimate()
{
    const float ESTIMATE_RADIUS_M = 0.5;
    set_colour(BLACK);
    Pose2D pose = kalman_.get_estimate();
    draw_circle(pose.x, pose.y, ESTIMATE_RADIUS_M);
}

void Viewer::set_colour(float r, float g, float b)
{
    glColor3f(r, g, b);
}
