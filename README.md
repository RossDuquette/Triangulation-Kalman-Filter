# Triangulation Kalman Filter

Simple application using OpenGL demonstrating a Kalman filter and triangulation.

Used to simulate a vehicle traversing through an environemnt with measurement
beacons. Gaussian noise is added to all distance measurements, and the angle
of vehicle can not be measured.

Kalman filter uses simple motion model and the average triangulation
measurement.

NOTE: Not optimized or the most accurate method of triangulation, but used as
a proof of concept.
