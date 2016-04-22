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


    for (int i = 0; i < 10; i++) {
        touches[i] = 0;
    }


    FILE *wts;
    char *mode = "r";
    wts = fopen("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/weights_Matrix.csv", mode);

    if (wts == NULL) {
        fprintf(stderr, "Can't open input file the weights.csv file!\n");
        exit(1);
    }

    float w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16;
    int i = 0;
    while (fscanf(wts, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",&w1,&w2,&w3,&w4,&w5,&w6,&w7,&w8,&w9,&w10,&w11,&w12,&w13,&w14,&w15,&w16) !=EOF){
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
        //weights[i][16] = w17;  /*,&w17,&w18,&w19,&w20*/
        //weights[i][17] = w18;
        //weights[i][18] = w19;  /*, w17, w18, w19, w20*/
        //weights[i][19] = w20;  ,%f,%f,%f,%f,%f
        i++;

    }

    fclose(wts);
    for (int i=0; i<4; i++){
        for (int j=0; j<20; j++){
            printf("%f", weights[i][j]);
        }
    }


    FILE *pos;
    char *modes = "r";
    pos = fopen("/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/position_size.csv", modes);

    if (pos == NULL) {
        fprintf(stderr, "Can't open input file the position_size.csv file!\n");
        exit(1);
    }

    float x, y, z, radius, length, size;
    int e = 0;
    while (fscanf(pos, "%f,%f,%f,%f,%f,%f\n",&x,&y,&z,&radius,&length,&size)==6){
        position_size[0][0] = x;
        position_size[0][1] = y;
        position_size[0][2] = z;
        position_size[0][3] = radius;
        position_size[0][4] = length;
        position_size[0][5] = size;

    }

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

    CreateCylinder(1, 4.4, 1.2, 0.,      0.4, 4.0, 'x');
    CreateCylinder(2, -4.4, 1.2, 0.,      0.4, 4.0, 'x');
    CreateCylinder(3, 8.4, 1.2, 4.0,     0.4, 4.0, 'z');
    CreateCylinder(4, -8.4, 1.2, 4.0,    0.4, 4.0, 'z');
    CreateCylinder(5, X, Y, Z, RADIUS, LENGTH, 'y');

    //CreateCylinder(5, 1.0, 0.0, 1.0, 1.0, 1.0, 'y');

    offsets[0] = 0;
    offsets[1] = 0;
    offsets[2] = 0;
    offsets[3] = 0;
    offsets[4] = 0;

    CreateHinge(0, 0, 1,    0.7, 1.2, 0.,      0.0, M_PI_2 , 0.0,     0.0, M_PI/2.0);

    CreateHinge(1, 0, 2,    -0.7, 1.2, 0.,      0.0, -M_PI_2, 0.0,     0.0, M_PI/2.0);

    CreateHinge(2, 1, 3,    8.4, 1.2, 0.,       0.0, M_PI_2, 0.0,     -M_PI/2.0, M_PI/2.0);

    CreateHinge(3, 2, 4,   -8.4, 1.2, 0.,       0.0, -M_PI_2, 0.0,     -M_PI/2.0, M_PI/2.0);



    //pause = !pause;
    clientResetScene();
}



#define TICKS_PER_DISPLAY 1001


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

            //for(int l=0; l < TICKS_PER_DISPLAY; l++) {     // COMMENT THIS OUT TO TURN GRAPHICS BACK ON


                m_dynamicsWorld->stepSimulation(ms / 900000.f);


                oneStep = !oneStep;


                if ( timeStep%1 == 0 ) {

                    update_ANN();
                    categorizer(SIZE, g_bar);
                    predicter();
                    g_var(guess_neuron);
                    g_var_vector[timeStep] = g_var_time;
                    m_vector[timeStep] = predict;


                }
                timeStep++;

                if (timeStep==1000){

                    Save_Categorize(categorize);
                    Save_Predict(predict);
                    uncertain();
                    Save_Uncertain(uncertainty);
                    printf("%f%f", categorize, uncertainty);
                    //printf("%f%f", categorize, uncertainty);
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
    joints[index]->setLimit(theta1,theta2);

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


void RagdollDemo::ActuateJoint2(int jointIndex, double desiredAngle,
                                double timeStep)
{
    double currentAngle;
    currentAngle = joints[jointIndex]->getHingeAngle();
    double maxImpulse = 1.0;

    joints[jointIndex]->setMaxMotorImpulse(btScalar(maxImpulse));
    double diff;
    diff = desiredAngle-(currentAngle+offsets[jointIndex]);
    joints[jointIndex]->enableAngularMotor(true, 2.5*diff, maxImpulse);
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
