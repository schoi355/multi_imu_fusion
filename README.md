# Attitude Estimation Code

This is the code of propagating attitude (in a unit quaternion form) given angular velocity.
See equation (102) of trawny tech report [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf) for details. (Beware: The reference uses JPL quaternion but the Eq. (102) holds for Hamiltonian quaternion as well.)

### How to build locally
```
$ cd src/
$ gcc example.c -lm
```
