/*************************************************************************/
// A simple car driver following a red road
/*
 have 2 front wheels
 */
/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

/*
 drive7.cpp:
 Drive using crude vision.
 Include obstacles
 
 Derived from ODE demo:
 demo_buggy.cpp: buggy with suspension.
 */

/*************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#if defined(WIN32) || defined(__CYGWIN__)// this prevents warnings when dependencies built
#include <windows.h>
#define M_PI 3.14159
#endif

#include <ode/odeconfig.h>
#include <OpenGL/gl.h>

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

/*************************************************************************/

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

/*************************************************************************/

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawSphere dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

/*************************************************************************/

// some constants
#define CHASSIS_LENGTH 2.667	// chassis length
#define CHASSIS_WIDTH 1.524	// chassis width
#define CHASSIS_HEIGHT 0.6604	// chassis height
#define CHASSIS_STARTZ 0.585	// starting height of chassis
#define CHASSIS_MASS 500.0	// chassis mass // cga: not realistic
/* CHASSIS MOI
 <ixx>114.946</ixx>
 <ixy>0.0</ixy>
 <ixz>0.0</ixz>
 <iyy>314.5424</iyy>
 <iyz>0.0</iyz>
 <izz>393.1444</izz>
 */
#define WHEEL_MASS 0.2	// wheel mass // cga: not realistic
#define WHEEL_RADIUS 0.254f // wheel radius
#define WHEEL_WIDTH 0.2286f // wheel width

#define MAX_SPEED (4.44/WHEEL_RADIUS)
// #define MAX_SPEED 1

#define BARREL_LENGTH 0.84
#define BARREL_RADIUS 0.29

#define VIEW_HEIGHT 1.0 // camera height

#define N_WHEELS 3
#define N_BARRELS 20

#define N_ROAD_SEGMENTS 100
#define ROAD_RADIUS 50.0 // fails at 2.5
#define ROAD_WIDTH 5.0

#define WINDOW_WIDTH 320
#define WINDOW_HEIGHT 240

#define MAX_N_PIXELS (WINDOW_WIDTH*WINDOW_HEIGHT)

// look ahead of the car hood.
#define IMAGE_LINE 50 // we wanted 30, but the car hood was in the way

#define ONLY_SEE_ONE_EDGE_FUDGE 120 // offset used when can only see one road
// edge

#define IMAGE_DIVIDER 4
#define CYCLES_PER_SECOND (IMAGE_DIVIDER*25) // 25 frames per second
#define TIME_STEP (1.0/((double) CYCLES_PER_SECOND))

#define STEERING_LIMIT 0.75
#define STEER_V_GAIN 10
#define STEERING_V_LIMIT 100.0

#define STEERING_GAIN (0.7854/160.0) // 45 degrees is 0.7854 radians is
// 160 pixels wide

/*************************************************************************/

// dynamics and collision objects (chassis, N_WHEELS wheels, environment)
static dWorldID world;
static dSpaceID space;
static dBodyID body[N_WHEELS+1];
static dJointID joint[N_WHEELS];
static dJointGroupID contactgroup;
static dGeomID ground;
static dSpaceID car_space;
static dGeomID box[1];
static dGeomID sphere[N_WHEELS];
static dGeomID barrel_left[N_BARRELS];
static dGeomID barrel_middle[N_BARRELS];
static dGeomID barrel_right[N_BARRELS];
// along road, road width, height
static dReal road_sides[3] = { ROAD_WIDTH, ROAD_WIDTH, 0.01 };
// multiple layers of roads to handle glitches.
static dGeomID road_segs1[N_ROAD_SEGMENTS];
static dGeomID road_segs2[N_ROAD_SEGMENTS];

// things that the user controls
static dReal speed_desired = 0;
static dReal steer_desired = 0;

static unsigned char my_image[MAX_N_PIXELS][3];

// output of vision
static int new_image = 0;
static int left_boundary, right_boundary;

static int oldSteerDes = 0;

/*************************************************************************/

/*
 double random_double( double min, double max )
 {
 return (double) ( scalbn( (double) random(), -31 )*(max-min) + min );
 }
 */

double random_double( double min, double max )
{
    double value;
    
    value = ((double) rand())/((double) RAND_MAX);
    return value*(max-min) + min;
}

/*************************************************************************/

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
    int i,n;
    
    // only collide things with the ground or barrels
    int g1 = (o1 == ground);
    int g2 = (o2 == ground);
    for ( i = 0; i < N_BARRELS; i++ )
    {
        g1 |= ( o1 == barrel_left[i] );
        g1 |= ( o1 == barrel_middle[i] );
        g1 |= ( o1 == barrel_right[i] );
        g2 |= ( o2 == barrel_left[i] );
        g2 |= ( o2 == barrel_middle[i] );
        g2 |= ( o2 == barrel_right[i] );
    }
    if (!(g1 ^ g2)) return;
    
    const int N = 20;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
    if (n > 0) {
        for (i=0; i<n; i++) {
            contact[i].surface.mode = // dContactSlip1 | dContactSlip2 |
            dContactMu2 |
            dContactSoftERP | dContactSoftCFM | dContactApprox1;
            // contact[i].surface.mu = dInfinity;
            // contact[i].surface.mu2 = dInfinity;
            contact[i].surface.mu = 100.0;
            contact[i].surface.mu2 = 50.0;
            // Gazebo has these as zero?
            contact[i].surface.slip1 = 0.1;
            contact[i].surface.slip2 = 0.1;
            /*
             contact[i].surface.soft_erp = 0.5;
             contact[i].surface.soft_cfm = 0.3;
             */
            contact[i].surface.soft_erp = 0.2;
            contact[i].surface.soft_cfm = 1e-7;
            dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
            dJointAttach (c,
                          dGeomGetBody(contact[i].geom.g1),
                          dGeomGetBody(contact[i].geom.g2));
        }
    }
    if ( n > N - 2 )
    {
        fprintf( stderr, "Too many contacts.\n" );
        exit( -1 );
    }
}

/*************************************************************************/

// set viewpoint
static void set_viewpoint()
{
    int i;
    float xyz[3];
    float hpr[3];
    double heading;
    dVector3 car_w;
    dVector3 camera_w;
    static int count = 0;
    
    // view management
    dBodyVectorToWorld( body[0], 1.0, 0.0, 0.0, car_w );
    heading = atan2( car_w[1], car_w[0] );
    hpr[0] = heading*(180/3.14);
    hpr[1] = 0.0f;
    hpr[2] = 0.0;
    
    dBodyGetRelPointPos( body[0], 0.0, 0.0, VIEW_HEIGHT, camera_w );
    for ( i = 0; i < 3; i++ )
        xyz[i] = camera_w[i];
    dsSetViewpoint (xyz,hpr);
}

/*************************************************************************/

// start simulation
static void start()
{
    dAllocateODEDataForThread(dAllocateMaskAll);
    
    set_viewpoint();
}

/*************************************************************************/

static void save_the_image( char *name )
{
    int i;
    FILE *stream = NULL;
    int x, y;
    int index;
    double intensity;
    int value;
    
    stream = fopen( name, "w" );
    if ( stream == NULL )
    {
        fprintf( stderr, "Cannot open %s\n", name );
        exit( -1 );
    }
    for ( y = WINDOW_HEIGHT - 1; y >= 0; y-- )
    {
        for ( x = 0; x < WINDOW_WIDTH; x++ )
        {
            index = y*WINDOW_WIDTH + x;
            intensity = 0;
            for ( i = 0; i < 3; i++ )
                intensity += my_image[ index ][i]*my_image[ index ][i];
            intensity = sqrt( intensity );
            value = 255.0*my_image[ index ][0]/intensity;
            if ( value > 255 )
                value = 255;
            fprintf( stream, "%d ", value );
        }
        fprintf( stream, "\n" );
    }
    fclose( stream );
}

/*************************************************************************/
//copied from the sense() function
/* LEGEND:
 *   0: no image
 *   1: stay straight
 *   2: go left
 *   3: go right
 */
static double senseBarrels() {
    double intensity;
    char name[10000];
    int i;
    int x, y;
    int index;
    double dvalue;
    double line[1000];
    double fline[1000];
    double fline2[1000];
    double a[3] = { 1.000000000000000, -1.561018075800718, 0.641351538057563 };
    double b[3] = { 0.020083365564211,  0.040166731128423, 0.020083365564211 };
    
    if ( new_image == 0 )
        return 0.0;
    
    y = IMAGE_LINE + 20; // line picked by CGA
    
    for ( x = 0; x < WINDOW_WIDTH; x++ )
    {
        index = y*WINDOW_WIDTH + x;
        intensity = 0;
        for ( i = 0; i < 3; i++ )
        {
            intensity += my_image[ index ][i]*my_image[ index ][i];
        }
        intensity = sqrt( intensity );
        dvalue = 255.0*my_image[ index ][0]/intensity;
        if ( dvalue > 255.0 )
            dvalue = 255.0;
        line[ x ] = dvalue;
    }
    
    // filter line
    fline[0] = line[0];
    fline[1] = line[1];
    for ( x = 2; x < WINDOW_WIDTH; x++ )
    {
        fline[ x ] = b[0]*line[ x ] + b[1]*line[ x - 1 ] + b[2]*line[ x - 2 ]
        - a[1]*fline[ x - 1 ] - a[2]*fline[ x - 2 ];
    }
    
    fline2[WINDOW_WIDTH - 1] = fline[WINDOW_WIDTH - 1];
    fline2[WINDOW_WIDTH - 2] = fline[WINDOW_WIDTH - 2];
    for ( x = WINDOW_WIDTH - 3; x >= 0; x-- )
    {
        fline2[ x ] = b[0]*fline[ x ] + b[1]*fline[ x + 1 ] + b[2]*fline[ x + 2 ]
        - a[1]*fline2[ x + 1 ] - a[2]*fline2[ x + 2 ];
    }
    
    // find left boundary
    left_boundary = -1;
    for ( x = 0; x < WINDOW_WIDTH; x++ )
    {
        if ( fline2[ x ] > 240.0 )
        {
            left_boundary = x;
            break;
        }
    }
    
    // find right boundary
    right_boundary = -1;
    for ( x = WINDOW_WIDTH - 1; x >= 0; x-- )
    {
        if ( fline2[ x ] > 240.0 )
        {
            right_boundary = x;
            break;
        }
    }
    
    //make an array to store events
    double events[20] = {0}; //make larger to store more objects; unnecessary for this assignment
    int eventIndex = 0;
    bool openRed = false;
    for(x = 0; x < WINDOW_WIDTH; x++) {
        if(fline2[x] > 240.0 && !openRed) {
            events[eventIndex] = double(x);
            eventIndex++;
            openRed = true;
        }
        else if(fline2[x] <= 240.0 && openRed) {
            events[eventIndex] = double(x);
            eventIndex++;
            openRed = false;
        }
    }
    
    //There is a barrel in the road
    if(eventIndex > 2) {
        double lengths[10] = {0};
        //calculate road gap sizes
        for(int i = 0; i < 18; i += 2) {
            lengths[i/2] = events[i+1] - events[i];
        }
        //find max road gap size
        double curMaxLen = 0;
        int curBestLeftIndex = 0;
        for(int i = 0; i < 10; i++) {
            if(lengths[i] > curMaxLen) {
                curMaxLen = lengths[i];
                curBestLeftIndex = i*2;
            }
        }
        printf("CURMAXLEN: %f, bestLeft: %d, rightmost: %d\n", curMaxLen, curBestLeftIndex, eventIndex-1);
        printf("ev: %f\n", double(events[eventIndex-1]));
        
        double left = (events[curBestLeftIndex] - events[0]) / (events[eventIndex-1] - events[0]);
        double right = (events[curBestLeftIndex+1] - events[0]) / (events[eventIndex-1] - events[0]);
        double target = (left+right)/2;
        
        printf("LEFT: %f, RIGHT: %f, TARGET: %f\n", left, right, target);
        return target;
    }
    return 0.5; //stay straight
}

// do sensing (in this case just vision)
static double sense()
{
    static int count = 0;
    double intensity;
    int value;
    char name[10000];
    int i;
    int x, y;
    int index;
    double dvalue;
    double line[1000];
    double fline[1000];
    double fline2[1000];
    double a[3] = { 1.000000000000000, -1.561018075800718, 0.641351538057563 };
    double b[3] = { 0.020083365564211,  0.040166731128423, 0.020083365564211 };
    
    if ( new_image == 0 )
        return 0;
    
    y = IMAGE_LINE; // line picked by CGA
    
    for ( x = 0; x < WINDOW_WIDTH; x++ )
    {
        index = y*WINDOW_WIDTH + x;
        intensity = 0;
        for ( i = 0; i < 3; i++ )
        {
            intensity += my_image[ index ][i]*my_image[ index ][i];
        }
        intensity = sqrt( intensity );
        dvalue = 255.0*my_image[ index ][0]/intensity;
        if ( dvalue > 255.0 )
            dvalue = 255.0;
        line[ x ] = dvalue;
    }
    
    // filter line
    fline[0] = line[0];
    fline[1] = line[1];
    for ( x = 2; x < WINDOW_WIDTH; x++ )
    {
        fline[ x ] = b[0]*line[ x ] + b[1]*line[ x - 1 ] + b[2]*line[ x - 2 ]
        - a[1]*fline[ x - 1 ] - a[2]*fline[ x - 2 ];
    }
    
    fline2[WINDOW_WIDTH - 1] = fline[WINDOW_WIDTH - 1];
    fline2[WINDOW_WIDTH - 2] = fline[WINDOW_WIDTH - 2];
    for ( x = WINDOW_WIDTH - 3; x >= 0; x-- )
    {
        fline2[ x ] = b[0]*fline[ x ] + b[1]*fline[ x + 1 ] + b[2]*fline[ x + 2 ]
        - a[1]*fline2[ x + 1 ] - a[2]*fline2[ x + 2 ];
    }
    
    // find left boundary
    left_boundary = -1;
    for ( x = 0; x < WINDOW_WIDTH; x++ )
    {
        if ( fline2[ x ] > 240.0 )
        {
            left_boundary = x;
            break;
        }
    }
    
    // find right boundary
    right_boundary = -1;
    for ( x = WINDOW_WIDTH - 1; x >= 0; x-- )
    {
        if ( fline2[ x ] > 240.0 )
        {
            right_boundary = x;
            break;
        }
    }
    
    double moveChoice = senseBarrels();
    
    if ( ( count % CYCLES_PER_SECOND ) == 0 )
    {
        printf( "vision: %d %d\n", left_boundary, right_boundary );
        sprintf( name, "f%03d", (int) (count/CYCLES_PER_SECOND) );
        // remove comment below to save an image every second
        // save_the_image( name );
    }
    
    count++;
    return moveChoice;
}

/*************************************************************************/

// robot controller
static void control(double barrelSense)
{
    static int count = 0;
    double target;
    
    // examples of how to access where car actually is
    /*
     const dReal *pos = dBodyGetPosition( body[0] );
     const dReal *quat = dBodyGetQuaternion( body[0] );
     const dReal *vel = dBodyGetLinearVel( body[0] );
     const dReal *omega = dBodyGetAngularVel( body[0] );
     */
    
    speed_desired = MAX_SPEED;
    // speed_desired = 0; // if you don't want the car to move
    
    // stop if tracking failed
    if ( left_boundary < 0 || right_boundary < 0 )
    {
        speed_desired = 0; // slam on the brakes
        count++;
        return;
    }
    
    if ( left_boundary > 0 && right_boundary < WINDOW_WIDTH - 1 )
        // can see both edges
        target = (left_boundary + right_boundary)/2;
    else if ( right_boundary >= WINDOW_WIDTH - 1 )
        // can only see left edge
        target = left_boundary + ONLY_SEE_ONE_EDGE_FUDGE;
    else
        // can only see right edge
        target = right_boundary - ONLY_SEE_ONE_EDGE_FUDGE;
    
    printf("\nTARGET: %f, SENSE: %f\n\n", target, barrelSense);

    float mult = 0.1;
    steer_desired = barrelSense * mult;

    int ratio = 9;
    
    if(barrelSense < 0.5) {
        int distToTarget = int(barrelSense * WINDOW_WIDTH);
        float subt = abs(float(distToTarget - left_boundary));
        printf("*********subt: %f\n left: %d\n", subt, left_boundary);
        if(subt > 0) {
            printf("Before: %f\n", steer_desired);
            steer_desired = steer_desired - ratio/subt;
            printf("After: %f\n", steer_desired);
        }
    }
    else if (barrelSense > 0.5) {
        int distToTarget = int(barrelSense * WINDOW_WIDTH);
        float subt = abs(float(right_boundary - distToTarget));
        printf("#########subt: %f\n", subt);
        if(subt > 0) {
            printf("Before: %f\n", steer_desired);
            steer_desired = steer_desired + ratio/subt;
            printf("After: %f\n", steer_desired);
        }
    }
    else {
        float oldRate = 1.5;
        float newRate = 1;
        steer_desired = ((oldRate * oldSteerDes) + newRate*STEERING_GAIN*(target - WINDOW_WIDTH/2))/(oldRate+newRate);
    }
    oldSteerDes = steer_desired;
    
    
    printf("OTHER: %f\n", STEERING_GAIN*(target - WINDOW_WIDTH/2));
    printf("STEER DESIRED: %f\n", steer_desired);
    
    if ( steer_desired > STEERING_LIMIT )
        steer_desired = STEERING_LIMIT;
    if ( steer_desired < -STEERING_LIMIT )
        steer_desired = -STEERING_LIMIT;
    
    if ( ( count % CYCLES_PER_SECOND ) == 0 )
        printf( "control: %g %g %g\n", target, steer_desired, speed_desired );

    count++;
}

/*************************************************************************/

static void simulate()
{
    int i;
    static int count = 0;
    
    const dReal *vel = dBodyGetLinearVel( body[0] );
    dReal chassis_speed = 0;
    
    for ( i = 0; i < 3; i++ )
        chassis_speed += vel[i]*vel[i];
    chassis_speed = sqrt( chassis_speed );
    
    // cga: note that speed is a commanded velocity and
    // steer is a steering wheel angle.
    dJointSetHinge2Param (joint[0],dParamVel2,-speed_desired);
    dJointSetHinge2Param (joint[0],dParamFMax2, 2000.0 );
    
    // steering
    dReal v = STEER_V_GAIN*(steer_desired - dJointGetHinge2Angle1 (joint[0]));
    if (v > STEERING_V_LIMIT) v = STEERING_V_LIMIT;
    if (v < -STEERING_V_LIMIT) v = -STEERING_V_LIMIT;
    
    if ( ( count % CYCLES_PER_SECOND ) == 0 )
        printf( "\nsim: %g %g %g; %g %g %g %g %g\n", steer_desired,
               dJointGetHinge2Angle1 (joint[0]),
               v, chassis_speed,
               dJointGetHinge2Angle1Rate (joint[0]),
               dJointGetHinge2Angle2Rate (joint[0]),
               dJointGetHinge2Angle2Rate (joint[1]),
               dJointGetHinge2Angle2Rate (joint[2]) );
    
    dJointSetHinge2Param (joint[0],dParamVel,v);
    dJointSetHinge2Param (joint[0],dParamFMax,100000.0);
    dJointSetHinge2Param (joint[0],dParamLoStop,-STEERING_LIMIT);
    dJointSetHinge2Param (joint[0],dParamHiStop,STEERING_LIMIT);
    dJointSetHinge2Param (joint[0],dParamFudgeFactor,0.1);
    
    dSpaceCollide (space,0,&nearCallback);
    dWorldStep( world, TIME_STEP );
    
    // remove all contact joints
    dJointGroupEmpty (contactgroup);
    
    count++;
}

/*************************************************************************/

static void graphics()
{
    int i;
    static int count = -1;
    
    count++;
    
    dsSetColor( 0, 1, 1 );
    dsSetTexture( DS_WOOD );
    dReal sides[3] = { CHASSIS_LENGTH, CHASSIS_WIDTH, CHASSIS_HEIGHT };
    dsDrawBox( dBodyGetPosition(body[0]), dBodyGetRotation(body[0]), sides );
    
//    FILE *f = fopen("/Users/eleynta/Desktop/d.txt", "a+");
//    fprintf(f, "POSITION: %f\n", *(dBodyGetPosition(body[0])));
//    fclose(f);
    
    dsSetColor (1,1,1);
    for (i=1; i<=N_WHEELS; i++) dsDrawCylinder( dBodyGetPosition(body[i]),
                                               dBodyGetRotation(body[i]),
                                               WHEEL_WIDTH, WHEEL_RADIUS );
    
    for ( i = 0; i < N_BARRELS; i++ )
    {
        dsSetTexture (DS_NONE);
        dsSetColor (0,1,0);
        dsDrawCylinder( dGeomGetPosition(barrel_left[i]),
                       dGeomGetRotation(barrel_left[i]),
                       BARREL_LENGTH, BARREL_RADIUS );
    }
    
    for ( i = 0; i < N_BARRELS; i++ )
    {
        dsSetTexture (DS_NONE);
        dsSetColor (0,0,1);
        dsDrawCylinder( dGeomGetPosition(barrel_middle[i]),
                       dGeomGetRotation(barrel_middle[i]),
                       BARREL_LENGTH, BARREL_RADIUS );
    }
    
    for ( i = 0; i < N_BARRELS; i++ )
    {
        dsSetTexture (DS_NONE);
        dsSetColor (0,1,1);
        dsDrawCylinder( dGeomGetPosition(barrel_right[i]),
                       dGeomGetRotation(barrel_right[i]),
                       BARREL_LENGTH, BARREL_RADIUS );
    }
    
    for ( i = 0; i < N_ROAD_SEGMENTS; i++ )
    {
        dsSetTexture (DS_NONE);
        dsSetColor (1,0,0);
        dsDrawBox( dGeomGetPosition(road_segs1[i]),
                  dGeomGetRotation(road_segs1[i]),
                  road_sides );
    }
    
    for ( i = 0; i < N_ROAD_SEGMENTS; i++ )
    {
        dsSetTexture (DS_NONE);
        dsSetColor (1,0,0);
        dsDrawBox( dGeomGetPosition(road_segs2[i]),
                  dGeomGetRotation(road_segs2[i]),
                  road_sides );
    }
    
    // only do at 25 Hz.
    if ( ( count % IMAGE_DIVIDER ) != 0 )
        return;
    
    // stash frame
    glReadPixels( 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE,
                 &my_image );
    
    new_image++;
}

/*************************************************************************/

// simulation loop
static void simLoop (int pause)
{
    
    if (!pause) {
        
        double s = sense();
        control(s);
        simulate();
        
        FILE *f = fopen("/Users/eleynta/Desktop/d.txt", "a+");
        fprintf(f, "%f 2\n", s);
        fclose(f);
    }
    
    set_viewpoint();
    graphics();
}

/*************************************************************************/

static void command (int cmd)
{
}

/*************************************************************************/

int main (int argc, char **argv)
{
    FILE *f = fopen("/Users/eleynta/Desktop/d.txt", "w+");
    fclose(f);

    int i, j;
    dMass m;
    double angle;
    dReal pad1[100];
    dReal R[12];
    dReal pad2[100];
    double barrel_offset;
    
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = 0;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    // create world
    dInitODE2(0);
    world = dWorldCreate();
    space = dHashSpaceCreate (0);
    contactgroup = dJointGroupCreate (0);
    dWorldSetGravity( world, 0, 0, -9.81 );
    ground = dCreatePlane (space,0,0,1,0);
    
    // chassis body
    body[0] = dBodyCreate (world);
    dBodySetPosition (body[0],0,0, CHASSIS_STARTZ );
    dMassSetBox (&m,1,CHASSIS_LENGTH,CHASSIS_WIDTH,CHASSIS_HEIGHT);
    dMassAdjust (&m,CHASSIS_MASS);
    dBodySetMass (body[0],&m);
    box[0] = dCreateBox (0,CHASSIS_LENGTH,CHASSIS_WIDTH,CHASSIS_HEIGHT);
    dGeomSetBody (box[0],body[0]);
    
    // wheel bodies
    for (i=1; i<=N_WHEELS; i++) {
        body[i] = dBodyCreate (world);
        dQuaternion q;
        dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
        dBodySetQuaternion (body[i],q);
        dMassSetSphere (&m,1,WHEEL_RADIUS);
        dMassAdjust (&m,WHEEL_MASS);
        dBodySetMass (body[i],&m);
        sphere[i-1] = dCreateSphere (0,WHEEL_RADIUS);
        dGeomSetBody (sphere[i-1],body[i]);
    }
    dBodySetPosition( body[1],
                     0.5*CHASSIS_LENGTH,
                     0,
                     CHASSIS_STARTZ-CHASSIS_HEIGHT*0.5 );
    dBodySetPosition( body[2],
                     -0.5*CHASSIS_LENGTH,
                     CHASSIS_WIDTH*0.5,
                     CHASSIS_STARTZ-CHASSIS_HEIGHT*0.5 );
    dBodySetPosition( body[3],
                     -0.5*CHASSIS_LENGTH,
                     -CHASSIS_WIDTH*0.5,
                     CHASSIS_STARTZ-CHASSIS_HEIGHT*0.5 );
    
    // front and back wheel hinges
    for (i=0; i<N_WHEELS; i++) {
        joint[i] = dJointCreateHinge2 (world,0);
        dJointAttach (joint[i],body[0],body[i+1]);
        const dReal *a = dBodyGetPosition (body[i+1]);
        dJointSetHinge2Anchor (joint[i],a[0],a[1],a[2]);
        dJointSetHinge2Axis1 (joint[i],0,0,1);
        dJointSetHinge2Axis2 (joint[i],0,1,0);
    }
    
    // set joint suspension
    for (i=0; i<N_WHEELS; i++) {
        dJointSetHinge2Param( joint[i],dParamSuspensionERP,0.2 );
        dJointSetHinge2Param( joint[i],dParamSuspensionCFM, 1e-4 );
    }
    
    // lock back wheels along the steering axis
    for (i=1; i<N_WHEELS; i++) {
        // set stops to make sure wheels always stay in alignment
        dJointSetHinge2Param (joint[i],dParamLoStop,0);
        dJointSetHinge2Param (joint[i],dParamHiStop,0);
        // the following alternative method is no good as the wheels may get out
        // of alignment:
        //   dJointSetHinge2Param (joint[i],dParamVel,0);
        //   dJointSetHinge2Param (joint[i],dParamFMax,dInfinity);
    }
    
    // create car space and add it to the top level space
    car_space = dSimpleSpaceCreate (space);
    dSpaceSetCleanup (car_space,0);
    dSpaceAdd (car_space,box[0]);
    for ( i = 0; i < N_WHEELS; i++ )
    {
        dSpaceAdd (car_space,sphere[i]);
    }
    
    // static obstacles
    for ( i = 0; i < N_BARRELS; i++ )
    {
        barrel_offset = ROAD_WIDTH/2 + BARREL_RADIUS;
        angle = 2*M_PI*((double) i)/((double) N_BARRELS);
        barrel_left[i] = dCreateCylinder( space, BARREL_RADIUS, BARREL_LENGTH );
        dGeomSetPosition( barrel_left[i],
                         (ROAD_RADIUS + barrel_offset)*sin( angle ),
                         (ROAD_RADIUS + barrel_offset)*cos( angle ) - ROAD_RADIUS,
                         BARREL_LENGTH/2 );
        /*
         dGeomSetPosition( barrel_left[i], N_BARRELS/2.0,
         -(i - N_BARRELS/2.0),
         BARREL_LENGTH/2 );
         */
        barrel_right[i] = dCreateCylinder( space, BARREL_RADIUS, BARREL_LENGTH );
        dGeomSetPosition( barrel_right[i],
                         (ROAD_RADIUS - barrel_offset)*sin( angle ),
                         (ROAD_RADIUS - barrel_offset)*cos( angle ) - ROAD_RADIUS,
                         BARREL_LENGTH/2 );
        barrel_middle[i] = dCreateCylinder( space, BARREL_RADIUS, BARREL_LENGTH );
        barrel_offset = random_double( -ROAD_WIDTH/2 + BARREL_RADIUS,
                                      ROAD_WIDTH/2 - BARREL_RADIUS );
        if ( i == 0 )
            barrel_offset = ROAD_WIDTH/2 + 2*BARREL_RADIUS;
        dGeomSetPosition( barrel_middle[i],
                         (ROAD_RADIUS + barrel_offset)*sin( angle ),
                         (ROAD_RADIUS + barrel_offset)*cos( angle ) - ROAD_RADIUS,
                         BARREL_LENGTH/2 );
    }
    
    // build the road
    road_sides[0] = 2*M_PI*(ROAD_RADIUS + ROAD_WIDTH)/N_ROAD_SEGMENTS;
    for ( i = 0; i < N_ROAD_SEGMENTS; i++ )
    {
        road_segs1[i] = dCreateBox( space,
                                   road_sides[0], road_sides[1], road_sides[2] );
        road_segs2[i] = dCreateBox( space,
                                   road_sides[0], road_sides[1], road_sides[2] );
        angle = 2*M_PI*((double) i)/((double) N_ROAD_SEGMENTS);
        dGeomSetPosition( road_segs1[i], ROAD_RADIUS*sin( angle ),
                         ROAD_RADIUS*cos( angle ) - ROAD_RADIUS, 0.005 );
        dGeomSetPosition( road_segs2[i], ROAD_RADIUS*sin( angle ),
                         ROAD_RADIUS*cos( angle ) - ROAD_RADIUS, 0.04 );
        // set up identity matrix
        for ( j = 0; j < 12; j++ )
            R[j] = 0;
        R[0] = R[5] = R[10] = 1.0;
        R[0] = cos( angle );
        R[1] = sin( angle );
        R[4] = -sin( angle );
        R[5] = cos( angle );
        dGeomSetRotation( road_segs1[i], R );
        dGeomSetRotation( road_segs2[i], R );
    }
    
    // run simulation
    dsSimulationLoop( argc, argv, WINDOW_WIDTH, WINDOW_HEIGHT, &fn );
    
    // clean up
    dGeomDestroy (box[0]);
    for ( i = 0; i < N_WHEELS; i++ )
    {
        dGeomDestroy (sphere[i]);
    }
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();
    return 0;
}

/*************************************************************************/
