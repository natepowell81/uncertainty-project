/*
 Bullet Continuous Collision Detection and Physics Library
 Ragdoll Demo
 Copyright (c) 2007 Starbreeze Studios
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 Written by: Marten Svanfeldt*/



#define CONSTRAINT_DEBUG_SIZE 0.2f

#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"
#include "math.h"
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;


double X, Y, Z, RADIUS, LENGTH, SIZE;
double position_size[1][6];
double movement;
double categorize, uncertainty, curr_predict[4];
double g_bar;
double current_angle;
double hidden_Neurons[4];
//double P_hat[4]; //predictive neurons
double Prop[4];     //Proprieceptive Neurons
double motor_neurons[4], ms;
float vision_neurons[4];
double desired_angle;
double guess_neuron[4], g_mean, g_var_time, g_var_vector[1000];
double m_mean, m_vector[1000];       //one for each timestep
double g_mean_uncertain, m_mean_uncertain;
double g_mean_vector[1000];
double numerator;
double denominator;
double denominator1;    //this and the numerator are for the uncertainty objective,
double denominator2;    //These 1 and 2 are for the two parts of the denominator that have to be calculated separately
double weights[4][26];
double Prop_current[4];
double Prop_previous[4];
float bias;
int touches[10];
double g_average[1000];
double previous_H[4];
btVector3 touchPoints[10];


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

class RagDoll
{
    
    btDynamicsWorld* m_ownerWorld;
    //btCollisionShape* m_shapes[BODYPART_COUNT];
    //btRigidBody* m_bodies[BODYPART_COUNT];
    //btTypedConstraint* m_joints[JOINT_COUNT];
    
    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
    {
        bool isDynamic = (mass != 0.f);
        
        btVector3 localInertia(0,0,0);
        if (isDynamic)
            shape->calculateLocalInertia(mass,localInertia);
        
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);
        
        m_ownerWorld->addRigidBody(body);
        
        return body;
    }
    
public:
    RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
    : m_ownerWorld (ownerWorld)
    {
        // Setup the geometry
        // m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
        
        // Setup all the rigid bodies
        btTransform offset; offset.setIdentity();
        offset.setOrigin(positionOffset);
        
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
        
        // m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);
        
        // Now setup the constraints
        // btHingeConstraint* hingeC;
        
        btTransform localA, localB;
        
        localA.setIdentity(); localB.setIdentity();
        localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
        localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
        //hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
        //hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
        //m_joints[JOINT_PELVIS_SPINE] = hingeC;
        //hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
        
        //m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);
    }
    
    virtual	~RagDoll ()
    {
        // int i;
        
        // Remove all constraints
        //		for ( i = 0; i < JOINT_COUNT; ++i)
        //		{
        //			m_ownerWorld->removeConstraint(m_joints[i]);
        //			delete m_joints[i]; m_joints[i] = 0;
        //		}
        //
        //		// Remove all bodies and shapes
        //		for ( i = 0; i < BODYPART_COUNT; ++i)
        //		{
        //			m_ownerWorld->removeRigidBody(m_bodies[i]);
        //
        //			delete m_bodies[i]->getMotionState();
        //
        //			delete m_bodies[i]; m_bodies[i] = 0;
        //			delete m_shapes[i]; m_shapes[i] = 0;
        //		}
    }
};

static RagdollDemo* ragdollDemo;

// bool RagdollDemo::myContactProcessedCallback(btManifoldPoint& cp,
bool myContactProcessedCallback(btManifoldPoint& cp,
                                void* body0, void* body1)
{
    int *ID1, *ID2;
    btCollisionObject* o1 = static_cast<btCollisionObject*>(body0);
    btCollisionObject* o2 = static_cast<btCollisionObject*>(body1);
    int groundID = 9;
    
    ID1 = static_cast<int*>(o1->getUserPointer());
    ID2 = static_cast<int*>(o2->getUserPointer());
    
    ragdollDemo->touches[*ID1] = 1;
    ragdollDemo->touches[*ID2] = 1;
    
    ragdollDemo->touchPoints[*ID1] = cp.m_positionWorldOnB;
    ragdollDemo->touchPoints[*ID2] = cp.m_positionWorldOnB;
    
    return false;
}


void RagdollDemo::initPhysics()
{
    
    
    ragdollDemo = this;
    gContactProcessedCallback = myContactProcessedCallback;
    
    for (int i = 0; i < 10; i++) {
        touches[i] = 0;
    }
    
    FILE *wts;
    const char *mode = "r";
    wts = fopen("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/weights_Matrix.csv", mode);
    
    if (wts == NULL) {
        fprintf(stderr, "Can't open input file the weights.csv file!\n");
        exit(1);
    }
    
    float w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24,w25,w26;
    int i = 0;
    while (fscanf(wts, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",&w1,&w2,&w3,&w4,&w5,&w6,&w7,&w8,&w9,&w10,&w11,&w12,&w13,&w14,&w15,&w16,&w17,&w18,&w19,&w20,&w21,&w22,&w23,&w24,&w25,&w26) !=EOF)
    {
        weights[i][0] = w1;
        weights[i][1] = w2;
        weights[i][2] = w3;
        weights[i][3] = w4;
        weights[i][4] = w5;
        weights[i][5] = w6;
        weights[i][6] = w7;
        weights[i][7] = w8;
        weights[i][8] = w9;
        weights[i][9] = w10;
        weights[i][10] = w11;
        weights[i][11] = w12;
        weights[i][12] = w13;
        weights[i][13] = w14;
        weights[i][14] = w15;
        weights[i][15] = w16;
        weights[i][16] = w17;  /*,&w17,&w18,&w19,&w20*/
        weights[i][17] = w18;
        weights[i][18] = w19;  /*, w17, w18, w19, w20*/
        weights[i][19] = w20;  //,%f,%f,%f,%f,%f
        weights[i][20] = w21;
        weights[i][21] = w22;
        weights[i][22] = w23;  /*,&w21,&w22,&w23,&w24*/
        weights[i][23] = w24;  /*, w21, w22, w23, w24*/
        weights[i][24] = w25;
        weights[i][25] = w26;
        i++;
    }
    
    fclose(wts);
    
    /*
     for (int i=0; i<4; i++){
     for (int j=0; j<25; j++){
     printf("%f", weights[i][j]);
     }
     }
     */
    
    /*
     for (int i=0; i<4; i++){
     for (int j=0; j<26; j++){
     weights[i][j] = -1.0;
     printf("%f", weights[i][j]);
     }
     }
     /*
     weights[0][0] = 1.0;
     weights[1][0] = 2.0;
     weights[2][0] = 3.0;
     weights[3][0] = 4.0;
     
     
     for (int i=0; i<4; i++){
     weights[i][4] = 1.0;
     weights[i][5] = 2.0;
     weights[i][6] = 3.0;
     weights[i][7] = 4.0;
     //printf("%f", weights[i][j]);
     }
     */
    
    
    
    
    FILE *pos;
    const char *modes = "r";
    pos = fopen("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/position_size.csv", modes);
    
    if (pos == NULL) {
        fprintf(stderr, "Can't open input file the position_size.csv file!\n");
        exit(1);
    }
    
    float x, y, z, radius, length, size;
    while (fscanf(pos, "%f,%f,%f,%f,%f,%f\n",&x,&y,&z,&radius,&length,&size)==6){
        position_size[0][0] = x;
        position_size[0][1] = y;
        position_size[0][2] = z;
        position_size[0][3] = radius;
        position_size[0][4] = length;
        position_size[0][5] = size;
        
    }
    
    //printf("%f%f%f%f%f%f", x, y, y, radius, length, size);
    
    fclose(pos);
    
    X = position_size[0][0];
    Y = position_size[0][1];
    Z = position_size[0][2];
    RADIUS = position_size[0][3];
    LENGTH = position_size[0][4];
    SIZE = position_size[0][5];
    
    //for (int j=0; j<6; j++){
    //    printf("%f", position_size[0][j]);
    //}
    
    
    bodyLookup[0] = 3;
    bodyLookup[1] = 4;
    bodyLookup[2] = 7;
    bodyLookup[3] = 8;
    
    timeStep = 0;
    timeStepExit = 0;
    
    for (int i=0; i<4; i++){
        previous_H[i] = 0.0;
    }
    
    
    // Setup the basic world
    
    setTexturing(true);
    setShadows(true);
    
    setCameraDistance(btScalar(12.));
    
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    
    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
    
    btVector3 worldAabbMin(-10000,-10000,-10000);
    btVector3 worldAabbMax(10000,10000,10000);
    m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);
    
    m_solver = new btSequentialImpulseConstraintSolver;
    
    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
    
    // Setup a big ground box
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
        m_collisionShapes.push_back(groundShape);
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0,-10,0));
        
        btCollisionObject* fixedGround = new btCollisionObject();
        fixedGround->setCollisionShape(groundShape);
        fixedGround->setWorldTransform(groundTransform);
        // m_dynamicsWorld->addCollisionObject(fixedGround, COL_POWERUP, powerupCollidesWith);
        m_dynamicsWorld->addCollisionObject(fixedGround);
        (fixedGround)->setUserPointer( &(IDs[5]) );
    }
    
    
    for (int i=0; i<10; i++) {
        IDs[i] = i;
    }
    
    
    
    CreateBox(0, 0., 0., 0., 1.0,      2.0, 0.5);
    
    CreateCylinder(1,   4.4, .9, 0.,      0.4, 4.0, 'x');
    CreateCylinder(2,  -4.4, .9, 0.,      0.4, 4.0, 'x');
    CreateCylinder(3,   8.4, .9, 4.0,     0.4, 4.0, 'z');
    CreateCylinder(4,  -8.4, .9, 4.0,     0.4, 4.0, 'z');
    
    //object
    CreateCylinder(5, X, Y, Z, RADIUS, LENGTH, 'y');
    
    //CreateCylinder(5, 1.0, 0.0, 1.0, 1.0, 1.0, 'y');
    
    offsets[0] = 0;
    offsets[1] = 0;
    offsets[2] = 0;
    offsets[3] = 0;
    offsets[4] = 0;
    
    //void RagdollDemo::CreateHinge(int index, int body1, int body2,
    //                              double x, double y, double z,
    //                              double ax, double ay, double az,
    //                              double theta1, double theta2)
    /*
     CreateHinge(0, 0, 1,    0.7, 1.2, 0.,      0.0, M_PI_2 , 0.0,     0.0, M_PI/2.0);
     
     CreateHinge(1, 0, 2,    -0.7, 1.2, 0.,      0.0, -M_PI_2, 0.0,     0.0, M_PI/2.0);
     
     CreateHinge(2, 1, 3,    8.4, 1.2, 0.,       0.0, M_PI_2, 0.0,     -M_PI/2.0, M_PI/2.0);
     
     CreateHinge(3, 2, 4,   -8.4, 1.2, 0.,       0.0, -M_PI_2, 0.0,     -M_PI/2.0, M_PI/2.0);
     */
    
    CreateHinge(0, 0, 1,    0.7, .9, 0.,      0.0, -M_PI_2 , 0.0);
    
    CreateHinge(1, 0, 2,    -0.7, .9, 0.,      0.0, M_PI_2, 0.0);//,     -2*M_PI, 2*M_PI);
    
    CreateHinge(2, 1, 3,    8.4, .9, 0.,       0.0, -M_PI_2, 0.0);
    
    CreateHinge(3, 2, 4,   -8.4, .9, 0.,       0.0, M_PI_2, 0.0);
    
    //pause = !pause;
    clientResetScene();
}



#define TICKS_PER_DISPLAY 1001


void RagdollDemo::clientMoveAndDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //simple dynamics world doesn't handle fixed-time-stepping
    float ms = getDeltaTimeMicroseconds();
    
    FILE *tim;
    const char *modess = "r";
    tim = fopen("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/timesteps.txt", modess);
    
    if  (tim == NULL) {
        fprintf(stderr, "Can't open timesteps file");
        exit(1);
    }
    int num_timesteps;
    int timesteps_import;
    while (fscanf(tim, "%d\n",&timesteps_import)==6){
        num_timesteps = timesteps_import;
    }
    fclose(tim);
    int index = 0;
    //printf("timesteps: %d", timesteps_import);
    
    float minFPS = 100000.f/60.f;
    if (ms > minFPS)
        ms = minFPS;
    
    //timeStepExit++;
    
    
    if (m_dynamicsWorld)
    {
        if (!pause || (pause && oneStep)) {
            
            //for(int l=0; l < TICKS_PER_DISPLAY; l++) {     // COMMENT THIS OUT TO TURN GRAPHICS BACK ON
                
                
                m_dynamicsWorld->stepSimulation(ms / 100000.f);
                
                
                oneStep = !oneStep;
                
                
                if ( timeStep%1 == 0 ) {
                    
                    movement = 0.0;
                    update_ANN();
                    
                    g_var(guess_neuron);
                    g_var_vector[index] = g_var_time;
                    m_vector[index] = movement;
                    g_mean_vector[index] = categorize;         //categorize is from g_mean_vector
                    //printf("time: %d", timesteps_import);
                    //printf("Movement: %f", movement);
                    //printf("\n==================\n");
                }
                timeStep++;
                index = index + 1;
                timesteps_import = 500;
                if (timeStep==timesteps_import){
                    Save_Categorize(categorize);
                    g_vector_save(g_var_vector);
                    guess_average_save(g_average);
                    m_vector_save(m_vector);
                    g_mean_vector_save(g_mean_vector);
                    //printf("Movement: %f", movement);
                    exit(0);
                    
                }
            }
            
            
            renderme();
            
            glFlush();
            
            glutSwapBuffers();
        //}                            //TO TURN GRAPHICS BACK ON COMMENT THIS OUT
    }
}


void RagdollDemo::CreateBox( int index, double x, double y, double z, double length, double height, double width)
{
    
    btVector3 localInertia(0.,0.,0.);
    
    btTransform offset;
    offset.setIdentity();
    offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
    
    btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
    
    geom[index] = new btBoxShape(btVector3(btScalar(length), btScalar(height), btScalar(width)));
    
    btRigidBody::btRigidBodyConstructionInfo rbInfo(btScalar(0.),myMotionState,geom[index],localInertia);
    body[index] = new btRigidBody(rbInfo);
    
    // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
    m_dynamicsWorld->addRigidBody(body[index]);
    
    (body[index])->setUserPointer( &(IDs[index]) );
}

// void RagdollDemo::CreateCylinder( int index, double x, double y, double xv, double yv, double zv, char orientation) {
void RagdollDemo::CreateCylinder( int index, double x, double y, double z, double r, double len, char orientation)
{
    
    // for some reason, giving them a bump to start makes them behave normally
    // btVector3 localInertia(1.5,0.,1.5);
    // btVector3 localInertia(.01,.0,.01);
    
    btVector3 localInertia(0.,0.,0.);
    
    btTransform offset; offset.setIdentity();
    offset.setOrigin(btVector3(btScalar(x), btScalar(y), btScalar(z)));
    
    // btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
    
    btScalar	mass(1.f);
    
    switch (orientation)
    {
            /*
             cylinder is defined as following:
             *
             * - principle axis aligned along y by default, radius in x, z-value not used
             * - for btCylinderShapeX: principle axis aligned along x, radius in y direction, z-value not used
             * - for btCylinderShapeZ: principle axis aligned along z, radius in x direction, y-value not used
             *
             */
        case 'x':
        {
            // offset.getBasis().setEulerZYX(btScalar(0), btScalar(0), btScalar(M_PI_2));
            btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
            // know that the x axis is principle, z axis not used
            // geom[index] = new btCylinderShapeX(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
            geom[index] = new btCylinderShapeX(btVector3(btScalar(len), btScalar(r), btScalar(0.)));
            // geom[index] = new btCapsuleShape(btScalar(r),btScalar(len));
            // geom[index] = new btCylinderShape(btVector3(btScalar(r), btScalar(len), btScalar(0.)));
            btCollisionShape* colShape = geom[index];
            colShape->calculateLocalInertia(mass,localInertia);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
            body[index] = new btRigidBody(rbInfo);
            // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
            m_dynamicsWorld->addRigidBody(body[index]);
            break;
        }
        case 'z':
        {
            // offset.getBasis().setEulerZYX(btScalar(M_PI_2), btScalar(0), btScalar(0));
            btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
            // geom[index] = new btCylinderShapeZ(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
            geom[index] = new btCylinderShapeZ(btVector3(btScalar(r), btScalar(0.), btScalar(len)));
            // geom[index] = new btCapsuleShape(btScalar(r),btScalar(len));
            // geom[index] = new btCylinderShape(btVector3(btScalar(r), btScalar(len), btScalar(0.)));
            btCollisionShape* colShape = geom[index];
            colShape->calculateLocalInertia(mass,localInertia);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
            body[index] = new btRigidBody(rbInfo);
            // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
            m_dynamicsWorld->addRigidBody(body[index]);
            break;
        }
        default:
            // offset.getBasis().setEulerZYX(btScalar(0), btScalar(M_PI_2), btScalar(0));
            btDefaultMotionState* myMotionState = new btDefaultMotionState(offset);
            // geom[index] = new btCapsuleShape(btScalar(r),btScalar(len));
            // geom[index] = new btCylinderShape(btVector3(btScalar(xv), btScalar(yv), btScalar(zv)));
            geom[index] = new btCylinderShape(btVector3(btScalar(r), btScalar(len), btScalar(0.)));
            btCollisionShape* colShape = geom[index];
            colShape->calculateLocalInertia(mass,localInertia);
            btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
            body[index] = new btRigidBody(rbInfo);
            // m_dynamicsWorld->addRigidBody(body[index], COL_LAND, landCollidesWith);
            m_dynamicsWorld->addRigidBody(body[index]);
    }
    (body[index])->setUserPointer( &(IDs[index]) );
}

void RagdollDemo::CreateHinge(int index, int body1, int body2,
                              double x, double y, double z,
                              double ax, double ay, double az)//,
//double theta1, double theta2)
{
    btVector3 p(x, y, z);
    btVector3 a(ax, ay, az);
    btVector3 p1 = PointWorldToLocal(body1, p);
    btVector3 p2 = PointWorldToLocal(body2, p);
    btVector3 a1 = AxisWorldToLocal(body1, a);
    btVector3 a2 = AxisWorldToLocal(body2, a);
    joints[index] = new btHingeConstraint(*body[body1], *body[body2],
                                          p1, p2,
                                          a1, a2, false);
    //joints[index]->setLimit(theta1,theta2);
    
    m_dynamicsWorld->addConstraint( joints[index] , true);
}

void RagdollDemo::ActuateJoint(int jointIndex, double desiredAngle,
                               double jointOffset, double timeStep)
{
    joints[jointIndex]->enableMotor(1);
    joints[jointIndex]->setMaxMotorImpulse(2);
    joints[jointIndex]->setMotorTarget(desiredAngle, 1);
}

void RagdollDemo::draw_line(btVector3 src, btVector3 dst, btVector3 color)
{
    glLineWidth(5.0);
    glColor3f(color.x(), color.y(), color.z());
    glBegin(GL_LINES);
    glVertex3f(src.x(), src.y(), src.z());
    glVertex3f(dst.x(), dst.y(), dst.z());
    glEnd();
}

void RagdollDemo::g_vector_save(double vector[1000]) {
    const int size = 1000; // change to 1000
    //double vector[10] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    
    ofstream outFile("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/g_var_vector.txt");
    if (outFile.is_open())
    {
        for (int count = 0; count < size; count ++){
            outFile << vector[count] << "\n";
        }
        outFile.close();
    }
    else
    {
        cout << "Unable to open file";
    }
}

void RagdollDemo::m_vector_save(double vector[1000]) {
    const int size = 1000; // change to 1000
    //double vector[10] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    
    ofstream outFile("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/m_vector.txt");
    if (outFile.is_open())
    {
        for (int count = 0; count < size; count ++){
            outFile << vector[count] << "\n";
        }
        outFile.close();
    }
    else
    {
        cout << "Unable to open file";
    }
}

void RagdollDemo::g_mean_vector_save(double vector[1000]) {
    const int size = 1000; // change to 1000
    //double vector[10] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    
    ofstream outFile("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/g_mean_vector.txt");
    if (outFile.is_open())
    {
        for (int count = 0; count < size; count ++){
            outFile << vector[count] << "\n";
        }
        outFile.close();
    }
    else
    {
        cout << "Unable to open file";
    }
}

void RagdollDemo::guess_average_save(double vector[1000]) {
    const int size = 1000; // change to 1000
    //double vector[10] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};
    
    ofstream outFile("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/g_average_vector.txt");
    if (outFile.is_open())
    {
        for (int count = 0; count < size; count ++){
            outFile << vector[count] << "\n";
        }
        outFile.close();
    }
    else
    {
        cout << "Unable to open file";
    }
}


double RagdollDemo::CalculateVision(int rays, double radius, double spread, btVector3 src, double azi, double zen)
{
    int count = 1;
    double distance = radius;
    btVector3 dst = btVector3(src.x() + radius * sin(zen) * sin(azi),
                              src.y() + radius * cos(zen),
                              src.z() - radius * sin(zen) * cos(azi));
    
    btCollisionWorld::ClosestRayResultCallback RayCallback(src, dst);
    m_dynamicsWorld->rayTest(src, dst, RayCallback);
    
    // Cast the center ray
    if (RayCallback.hasHit())
    {
        distance = sqrt(pow(src.x() - RayCallback.m_hitPointWorld.getX(), 2) +
                        pow(src.y() - RayCallback.m_hitPointWorld.getY(), 2) +
                        pow(src.z() - RayCallback.m_hitPointWorld.getZ(), 2));
        if (graphics_) draw_line(src, dst, btVector3(0.0, 0.0, 0.0));
    }
    else if (graphics_) draw_line(src, dst, btVector3(0.75, 0.75, 0.75));
    
    // Cast the other rays in increasing spread around the target
    for (int i = 1; i < rays; ++i)
    {
        ++count;
        dst.setZ(src.z() - radius * sin(zen) * cos(azi + pow(-1, i) * ((int) ((i + 1) / 2)) * spread));
        dst.setY(src.y() + radius * cos(zen));
        dst.setX(src.x() + radius * sin(zen) * sin(azi + pow(-1, i) * ((int) ((i + 1) / 2)) * spread));
        btCollisionWorld::ClosestRayResultCallback RayCallback(src, dst);
        m_dynamicsWorld->rayTest(src, dst, RayCallback);
        if (RayCallback.hasHit())
        {
            distance += sqrt(pow(src.x() - RayCallback.m_hitPointWorld.getX(), 2) +
                             pow(src.y() - RayCallback.m_hitPointWorld.getY(), 2) +
                             pow(src.z() - RayCallback.m_hitPointWorld.getZ(), 2));
            if (graphics_) draw_line(src, dst, btVector3(0.2, 0.2, 0.2));
        }
        else
        {
            distance += radius;
            if (graphics_) draw_line(src, dst, btVector3(1.0, 1.0, 1.0));
        }
    }
    return (distance / radius / count) * 2. - 1.;
}

void RagdollDemo::update_ANN() {
    //printf("%ld", timeStep);
    clear_neurons();
    double sensors[8];
    
    vision_neurons[0] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 2.1, 7.8);  //
    //vision_neurons[1] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 2.4, 7.8);
    vision_neurons[1] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 2.8, 7.8);  //
    //vision_neurons[3] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 3.0, 7.8);
    //vision_neurons[2] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 3.3, 7.8);
    vision_neurons[2] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 3.5, 7.8);  //
    //vision_neurons[3] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 3.9, 7.8);
    vision_neurons[3] = CalculateVision(1., 15., M_PI_4, btVector3 (0.0, 1.4, 0.2), 4.2, 7.8);  //
    
    for (int i=0; i<8; i++){
        //printf("%f", tanh(vision_neurons[i]));
    }
    
    if (timeStep==1){
        for (int i=0;i<4;i++){
            Prop_previous[i] = 0.0;
        }
    }
    
    for (int i = 0; i < 4; i++){
        Prop[i] = (joints[i]->getHingeAngle());
        Prop[i] = tanh(Prop[i]);
        Prop_current[i] = Prop[i];
        //printf("Current: %f Prevoius: %f", Prop_current, Prop_previous);
        
    }
    
    
    for (int i=0;i<4;i++){
        movement1(Prop_current, Prop_previous);
        Prop_previous[i] = Prop_current[i];
    }
    
    
    
    for (int i=0;i<4;i++){
        sensors[i] = vision_neurons[i];
        sensors[i+4] = Prop[i];
    }
    
    for (int i=0;i<8;i++){
        //printf("Sensors: %f", sensors[i]);
    }
    
    bias = 1.0;
    //printf("bias: %f", bias);
    //printf("\n==================\n");
    for (int h=0; h<4; h++) {
        hidden_Neurons[h] = 0.0;
        
        hidden_Neurons[h] = bias * weights[h][25];
        //printf("H%d: %f,", h, hidden_Neurons[h]);
        
        
        for (int p=0; p<8; p++){
            hidden_Neurons[h] = hidden_Neurons[h] + sensors[p] * weights[h][p];
            //printf("H%d: %f,", h, hidden_Neurons[h]);
        }
        
        for (int p=0; p<4; p++){
            hidden_Neurons[h] = hidden_Neurons[h] + previous_H[p]*weights[h][p+12];
            //printf("H%d: %f,", h, hidden_Neurons[h]);
        }
        
        //for (int p=0; p<4; p++){
        //    hidden_Neurons[h] = hidden_Neurons[h] + tanh(vision_neurons[p]) * weights[h][p+16];
        //}
        
        //for (int p=0; p<4; p++){
        //    hidden_Neurons[h] = hidden_Neurons[h] + previous_H[p]*weights[h][p+12];  //recurrent
        // }
        //hidden_Neurons[h] = tanh(hidden_Neurons[h]);
    }
    
    
    g_bar = 0.0;
    for (int i=0; i<4; i++) {
        for (int p=0; p<4; p++) {
            guess_neuron[i] = guess_neuron[i] + (hidden_Neurons[p] * weights[p][i+16]);
        }
        guess_neuron[i] = tanh(guess_neuron[i]);
        
        g_bar = g_bar + guess_neuron[i];
    }
    g_bar = g_bar / 4.0;    //mean of guess neurons at each time step
    
    g_average[timeStep] = g_bar;
    //printf("Guess Average: %f", g_bar);
    
    //printf("\n==================\n");
    for (int i=0; i<4; i++) {
        //printf("P%d: %f,", i, Prop[i]);
        //printf("H%d: %f,", i, hidden_Neurons[i]);
        //printf("%f", guess_neuron[i]);
    }
    /*
     printf("\n==================\n");
     for (int i=0; i<4; i++) {
     printf("PH%d: %f,", i, previous_H[i]);
     //printf("%f", guess_neuron[i]);
     }
     
     printf("\n==================\n");
     */
    
    
    for (int h=0; h<4; h++){
        for (int p=0; p<4; p++) {
            motor_neurons[h] = motor_neurons[h] + (hidden_Neurons[p] * weights[p][h+8]);
        }
        
        //motor_neurons[h] = (tanh(motor_neurons[h]));
        //motor_neurons[h] = motor_neurons[h]*(M_PI_2);
        //ActuateJoint2(h, motor_neurons[h], ms / 900000.f);
        
        
        if (h==0){
            motor_neurons[h] = (tanh(motor_neurons[h]));
            motor_neurons[h] = -fabs(motor_neurons[h])*(M_PI_2);
            ActuateJoint2(h, motor_neurons[h], ms / 100000.f);
        }
        if (h==1){
            motor_neurons[h] = (tanh(motor_neurons[h]));
            motor_neurons[h] = -fabs(motor_neurons[h])*(M_PI_2);
            ActuateJoint2(h, motor_neurons[h], ms / 100000.f);
        }
        if (h==2){
            motor_neurons[h] = (tanh(motor_neurons[h]));
            motor_neurons[h] = motor_neurons[h]*(M_PI_2);
            ActuateJoint2(h, motor_neurons[h], ms / 100000.f);
        }
        if (h==3){
            motor_neurons[h] = (tanh(motor_neurons[h]));
            motor_neurons[h] = motor_neurons[h]*(M_PI_2);
            ActuateJoint2(h, motor_neurons[h], ms / 100000.f);
        }
        
        //printf(" M%d: %f", h, motor_neurons[h]);
    }
    
    
    for (int i=0; i<4; i++){
        previous_H[i] = hidden_Neurons[i];
    }
    
    categorizer();
    //predicter();
}

void RagdollDemo::categorizer() {
    g_mean = 0.0;
    for (int i=0; i<4; i++){
        g_mean = g_mean + pow((guess_neuron[i] - SIZE), 2);
    }
    g_mean = g_mean/4.0;
    categorize = g_mean;
    //printf("categorize error: %f", categorize );
}

void RagdollDemo::Save_Categorize(double categorize) {
    
    FILE *cat;
    char outputFilename[] = "/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/categorize.csv";
    cat = fopen(outputFilename, "w");
    
    if (cat == NULL) {
        fprintf(stderr, "Can't open file %s\n",
                outputFilename);
        exit(1);
    }
    
    fprintf(cat, "%f\n", categorize);
    fclose(cat);
}

void RagdollDemo::Save_Predict(double predict) {
    
    FILE *pre;
    char outputFilename[] = "/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/predict.csv";
    pre = fopen(outputFilename, "w");
    
    if (pre == NULL) {
        fprintf(stderr, "Can't open file %s\n",
                outputFilename);
        exit(1);
    }
    
    //fprintf(pre, "%f\n", predict);
    fclose(pre);
}

void RagdollDemo::Save_movement(double predict[1000]) {
    
    FILE *mov;
    char outputFilename[] = "/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/movement.csv";
    mov = fopen(outputFilename, "w");
    
    if (mov == NULL) {
        fprintf(stderr, "Can't open file %s\n",
                outputFilename);
        exit(1);
    }
    
    fprintf(mov, "%f\n", predict[1000]);
    fclose(mov);
}

void RagdollDemo::clear_neurons() {
    for (int i=0; i <4; i++) {
        Prop[i] = 0.0;
        hidden_Neurons[i] = 0.0;
        //P_hat[i] = 0.0;
        guess_neuron[i] = 0.0;
        motor_neurons[i] = 0.0;
    }
    for (int i=0; i<4; i++){
        vision_neurons[i] = 0.0;
    }
}

void RagdollDemo::g_var(double guess_neuron[4]) {
    g_mean = 0.0;
    for (int i=0; i<4; i++){
        g_mean = g_mean + guess_neuron[i];
    }
    g_mean = g_mean / 4.0;
    //printf("%f", g_mean);
    
    g_var_time = 0.0;
    for (int j=0; j<4; j++){
        g_var_time = g_var_time + pow((guess_neuron[j] - g_mean), 2);
    }
    g_var_time = g_var_time / 3.0;
    //printf("%f", g_var_time);
}

void RagdollDemo::uncertain() {
    uncertainty = 0.0;
    g_mean_uncertain = 0.0;
    m_mean_uncertain = 0.0;
    numerator = 0.0;
    denominator1 = 0.0;
    denominator2 = 0.0;
    
    for (int i=0; i<1000; i++){
        g_mean_uncertain = g_mean_uncertain + g_var_vector[i];
        m_mean_uncertain = m_mean_uncertain + m_vector[i];
        //printf("%f", g_mean_uncertain);
        //printf("%f", m_mean_uncertain);
    }
    
    g_mean_uncertain = g_mean_uncertain/1000.0;
    m_mean_uncertain = m_mean_uncertain/1000.0;
    
    for (int j=0; j<1000; j++){
        
        numerator = numerator + ((g_var_vector[j] - g_mean_uncertain) * (m_vector[j] - m_mean_uncertain));
        denominator1 = denominator1 + sqrt(pow((g_var_vector[j] - g_mean_uncertain), 2));
        denominator2 = denominator2 + sqrt(pow((m_vector[j] - m_mean_uncertain), 2));
    }
    
    denominator = denominator1 * denominator2;
    uncertainty = numerator / denominator;
    //printf("%f", numerator);
    //printf("%f", denominator1);
    uncertainty = pow(uncertainty, 2.0);
}


void RagdollDemo::movement1(double Prop_current[4], double Prop_previous[4]) {                     //THIS IS M FROM THE OBJECTIVE FUNCTION
    for (int i=0; i<4; i++){
        //current_angle = (joints[i]->getHingeAngle());
        //desired_angle = motor_neurons[i];
        curr_predict[i] = fabs(Prop_current[i] - Prop_previous[i]);  //took out the 1000
        //printf("%f", current_angle);
        //printf("%f", Prop_previous);
    }
    for (int i=0; i<4; i++){
        movement = movement + curr_predict[i];
    }
    movement = movement / 4.0;
    //printf("Current: %f Previous: %f", Prop_current, Prop_previous);
    //printf("\n==================\n");
}

void RagdollDemo::ActuateJoint2(int jointIndex, double desiredAngle,
                                double timeStep)
{
    double currentAngle;
    currentAngle = joints[jointIndex]->getHingeAngle();
    double maxImpulse = 2.;
    
    joints[jointIndex]->setMaxMotorImpulse(btScalar(maxImpulse));
    double diff;
    diff = desiredAngle - (currentAngle+offsets[jointIndex]);
    //printf("%f", diff);
    joints[jointIndex]->enableAngularMotor(true, 5.5*diff, maxImpulse);
}

void RagdollDemo::displayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    renderme();
    
    //optional but useful: debug drawing
    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();
    
    glFlush();
    glutSwapBuffers();
}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'e':
        {
            btVector3 startOffset(0,2,0);
            break;
        }
        case 'p':
        {
            pause = !pause;
            break;
        }
        case 'o':
        {
            oneStep = !oneStep;
            break;
        }
        default:
            DemoApplication::keyboardCallback(key, x, y);
    }
    
    
}

void RagdollDemo::exitPhysics()
{
    DeleteObject(0);
    
    int i;
    
    //cleanup in the reverse order of creation/initialization
    
    //remove the rigidbodies from the dynamics world and delete them
    
    for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }
    
    //delete collision shapes
    for (int j=0;j<m_collisionShapes.size();j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    
    //delete dynamics world
    delete m_dynamicsWorld;
    
    //delete solver
    delete m_solver;
    
    //delete broadphase
    delete m_broadphase;
    
    //delete dispatcher
    delete m_dispatcher;
    
    delete m_collisionConfiguration;
    
}


