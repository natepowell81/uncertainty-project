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
using namespace std;

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
    
    
    FILE *wts;
    char *mode = "r";
    wts = fopen("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/weights_Matrix.csv", mode);
    
    if (wts == NULL) {
        fprintf(stderr, "Can't open input file the weights.csv file!\n");
        exit(1);
    }
    
    float w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13;
    int i = 0;
    while (fscanf(wts, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",&w1,&w2,&w3,&w4,&w5,&w6,&w7,&w8,&w9,&w10,&w11,&w12,&w13) !=EOF){
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
        i++;
        
    }
    
    fclose(wts);
    for (int i=0; i<4; i++){
        for (int j=0; j<13; j++){
            printf("%f", weights[i][j]);
        }
    }
    
    
    
    
    bodyLookup[0] = 3;
    bodyLookup[1] = 4;
    bodyLookup[2] = 7;
    bodyLookup[3] = 8;
    
    timeStep = 0;
    timeStepExit = 0;
    
    
    
    // Setup the basic world
    
    setTexturing(true);
    setShadows(true);
    
    
    m_cameraTargetPosition = btVector3(0., 0., 10.);
    setCameraDistance(btScalar(20.));
    setEle(btScalar(60.0));
    
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
        (fixedGround)->setUserPointer( &(IDs[9]) );
    }
    
    
    for (int i=0; i<10; i++) {
        IDs[i] = i;
    }
    double verticaloffset = 0.0;
    
    
    CreateBox(0, 0., 0., 0., 0.1,      0.1, 0.1);
    
    CreateCylinder(0, 2.0, 0., 0.,      0.2, 2.0, 'x');
    CreateCylinder(2, -2., 0., 0.,     0.2, 2.0, 'x');
    CreateCylinder(3, 4.0, 0., 2.1,     0.2, 2.0, 'z');
    CreateCylinder(4, -4.0, 0., 2.1,    0.2, 2.0, 'z');
    
    offsets[0] = 0;
    offsets[1] = 0;
    offsets[2] = 0;
    offsets[3] = 0;
    offsets[4] = 0;
    
    CreateHinge(0, 0, 1,    0.1, 0.1, 0.,      0.0, 1.0, 0.0,   -(90.+offsets[0])*3.14159/180., 0.0);
    
    CreateHinge(1, 0, 2,   -0.1, 0.1, 0.,      0.0, 1.0, 0.0,   0.0, (90.+offsets[0])*3.14159/180.);
    
    CreateHinge(2, 1, 3,    4.0, 0.1, 0.,       0.0, 1.0, 0.0,   -(90.+offsets[0])*3.14159/180.,(90.+offsets[0])*3.14159/180.);
    
    CreateHinge(3, 2, 4,   -4.0, 0.1, 0.,       0.0, 1.0, 0.0,   -(90.+offsets[0])*3.14159/180.,-(90.+offsets[0])*3.14159/180.);
    
    joints[0]->setLimit(-M_PI, 0);
    joints[1]->setLimit(-M_PI, 0);
    joints[2]->setLimit(-M_PI, 0);
    joints[3]->setLimit(-M_PI, 0);
    
    
    //pause = !pause;
    clientResetScene();
}

void RagdollDemo::clientMoveAndDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //simple dynamics world doesn't handle fixed-time-stepping
    float ms = getDeltaTimeMicroseconds();
    
    float minFPS = 900000.f/60.f;
    if (ms > minFPS)
        ms = minFPS;
    
    timeStepExit++;
    
    if (m_dynamicsWorld)
    {
        if (!pause || (pause && oneStep)) {
            for (int i=0; i<10; i++) {
                touches[i] = 0;
            }
            
            {
                m_dynamicsWorld->stepSimulation(ms / 900000.f);
            }
            
            oneStep = !oneStep;
            
            //            double knees,bodyj;
            //            knees = 45;
            //            bodyj = 45;
            //            ActuateJoint2(0, knees, ms / 1000000.f);
            //            ActuateJoint2(1, knees, ms / 1000000.f);
            //            ActuateJoint2(2, knees, ms / 1000000.f);
            //            ActuateJoint2(3, knees, ms / 1000000.f);
            //            ActuateJoint2(4, bodyj, ms / 1000000.f);
            //            ActuateJoint2(5, bodyj, ms / 1000000.f);
            //            ActuateJoint2(6, bodyj, ms / 1000000.f);
            //            ActuateJoint2(7, bodyj, ms / 1000000.f);
            
            //ActuateJoint2(0, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            //ActuateJoint2(1, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            //ActuateJoint2(2, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            //ActuateJoint2(3, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            //ActuateJoint2(4, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            //ActuateJoint2(5, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            //ActuateJoint2(6, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            //ActuateJoint2(7, (rand()/double(RAND_MAX))*90.-45, ms / 1000000.f);
            
            if ( timeStep%10 == 0 ) {
                
                for (int i = 0; i < 4; ++i)
                    Prop[0][i] = (joints[i]->getHingeAngle() + M_PI_2) / M_PI_2;
                
                for (int h=0; h<4; h++) {
                    hidden_Neurons[0][h] = 0.0;
                    for (int p=0; p<4; p++){
                        hidden_Neurons[0][h] = tanh( hidden_Neurons[0][h] + weights[p][h] * Prop[0][p] );
                    }
                }
                
                for (int h=0; h<4; h++){
                    P_hat[0][h] = P_hat[0][h] + hidden_Neurons[0][h] * weights[h][h+4];
                }
                
                for (int i=0; i<4; i++) {
                    guess_neuron = guess_neuron + hidden_Neurons[0][i] * weights[i][12];
                }
            }
            
            //for (int i=0; i<4; i++) {
            //    printf("%f", Prop[0][i]);
            //    printf("%f", hidden_Neurons[0][i]);
            //    printf("%f", P_hat[0][i]);
            //    printf("%f", guess_neuron);
            //}
            
            for (int h=0; h<4; h++){
                double motorCommand = 0.0;
                for (int p=0; p<4; p++){
                    motorCommand = motorCommand + hidden_Neurons[0][h] * weights[h][p+8];
                    motorCommand = tanh (motorCommand);
                    motorCommand = motorCommand*90.;
                    ActuateJoint2(h, motorCommand, ms / 900000.f);
                }
            }
            timeStep++;
            
            //oneStep = false;
            
            
        }
        //if (timeStep==1000){
        //    Save_Position(body[0]);
        //    exit(0);
        //}
    }
    
    
    renderme();
    
    glFlush();
    
    glutSwapBuffers();
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
                              double ax, double ay, double az,
                              double theta1, double theta2)
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
    if (index < 8)
    {
        joints[index]->setLimit(theta1,theta2);
    }
    m_dynamicsWorld->addConstraint( joints[index] , true);
}

void RagdollDemo::ActuateJoint(int jointIndex, double desiredAngle,
                               double jointOffset, double timeStep)
{
    joints[jointIndex]->enableMotor(1);
    joints[jointIndex]->setMaxMotorImpulse(2);
    joints[jointIndex]->setMotorTarget(desiredAngle, 1);
}

void RagdollDemo::ActuateJoint2(int jointIndex, double desiredAngle,
                                double timeStep)
{
    double currentAngle;
    currentAngle = joints[jointIndex]->getHingeAngle();
    double maxImpulse = 1.50;
    //double maxForce = 40.0;
    joints[jointIndex]->setMaxMotorImpulse(btScalar(maxImpulse));
    double diff;
    diff = desiredAngle-(currentAngle+offsets[jointIndex]);
    joints[jointIndex]->enableAngularMotor(true, btScalar(5.0*diff), btScalar(maxImpulse));
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