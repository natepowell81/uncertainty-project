/*
 Bullet Continuous Collision Detection and Physics Library
 RagdollDemo
 Copyright (c) 2007 Starbreeze Studios
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 
 Written by: Marten Svanfeldt
 */

/*
 Bullet Continuous Collision Detection and Physics Library
 RagdollDemo
 Copyright (c) 2007 Starbreeze Studios
 
 This software is provided 'as-is', without any express or implied warranty.
 In no event will the authors be held liable for any damages arising from the use of this software.
 Permission is granted to anyone to use this software for any purpose,
 including commercial applications, and to alter it and redistribute it freely,
 subject to the following restrictions:
 
 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
 3. This notice may not be removed or altered from any source distribution.
 
 Written by: Marten Svanfeldt
 */


#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btDefaultMotionState.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "GLDebugDrawer.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include <string>
#include <iostream>


class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class RagdollDemo : public GlutDemoApplication
{
    //keep the collision shapes, for deletion/cleanup
    
    btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;
    
    btBroadphaseInterface*	m_broadphase;
    
    btCollisionDispatcher*	m_dispatcher;
    
    btConstraintSolver*	m_solver;
    
    btDefaultCollisionConfiguration* m_collisionConfiguration;
    
    btRigidBody* body[5]; // one main body, 4x2 leg segments
    
    btCollisionShape* geom[5];
    
    btHingeConstraint* joints[4];
    
    std::string id;  //id number
    
    bool oneStep;
    
    int bodyLookup[4];
    
    bool pause;
    bool graphics_ = true;
    double offsets[8];
    int IDs[6];
    
public:
    //double size;
    /*double categorize, uncertainty, curr_predict[4];
    double g_bar;
    double current_angle;
    btVector3 touchPoints[10];
    double hidden_Neurons[4];
    //double P_hat[4]; //predictive neurons
    double Prop[4];     //Proprieceptive Neurons
    double motor_neurons[4], ms;
    float vision_neurons[8];
    double desired_angle;
    double guess_neuron[4], g_mean, g_var_time, g_var_vector[1000];
    double m_mean, m_vector[1000];       //one for each timestep
    double g_mean_uncertain, m_mean_uncertain;
    double numerator;
    double denominator;
    double denominator1;    //this and the numerator are for the uncertainty objective,
    double denominator2;    //These 1 and 2 are for the two parts of the denominator that have to be calculated separately
    double weights[4][26];
    */
    int touches[10];
    btVector3 touchPoints[10];
    float randomNum;
    long timeStep;
    long timeStepExit;
    void initPhysics();
    void exitPhysics();
    
    /*
    RagdollDemo(std::string id_in){
        id = id_in;
    }
    */
    
    virtual void setid(std::string id_in)
    {
        id = id_in;
        //std::cout << "I AM A TEST" << id << std::endl;
    }
    
    virtual ~RagdollDemo()
    
    {
        exitPhysics();
    }
    
    virtual void clientMoveAndDisplay();
    
    virtual void displayCallback();
    
    virtual void keyboardCallback(unsigned char key, int x, int y);
    
    
    static DemoApplication* Create()
    {
        RagdollDemo* demo = new RagdollDemo();
        demo->myinit();
        demo->initPhysics();
        return demo;
    }
    
    float RandomNumber(float Min, float Max)
    {
        return ((float(rand()) / float(RAND_MAX)) * (Max - Min)) + Min;
    }
    
    //void CreateCapsule(int index, double x, double y, double z,
    //                                double length, double radius,
    //                                double anglex, double angley, double anglez,
    //                                double mass, double friction, double rfriction)
    //{
    //    geom[index] = new btCapsuleShape(radius, length);
    //
    //    btVector3 pos = btVector3(x, y, z);
    //    btTransform offset_trans;
    //    offset_trans.setIdentity();
    //    offset_trans.setOrigin(pos);
    //    offset_trans.getBasis().setEulerZYX(anglez, angley, anglex);
    //    btDefaultMotionState* myMotionState = new btDefaultMotionState(offset_trans);
    //    btVector3 localInertia(0,0,0);
    //    if (mass != 0.f) geom[index]->calculateLocalInertia(mass, localInertia);
    //    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, geom[index], localInertia);
    //    rbInfo.m_additionalDamping = true;
    //    body[index] = new btRigidBody(rbInfo);
    //    body[index]->setFriction(friction);
    //    body[index]->setRollingFriction(rfriction);
    //    m_dynamicsWorld->addRigidBody(body[index]);
    //}
    
    void draw_line(btVector3 src, btVector3 dst, btVector3 color);
    
    void CreateBox( int index, double x, double y, double z, double length, double height, double width);
    
    // void CreateCylinder( int index, double x, double y, double z, double xv, double yv, double zv, char orientation);
    void CreateCylinder( int index, double x, double y, double z, double r, double len, char orientation);

    void CreateHinge(int index, int body1, int body2, double x, double y, double z, double ax, double ay, double az);
    //, double theta1, double theta2
    void DeleteObject( int index ) {
        m_dynamicsWorld->removeRigidBody( body[index] );
        delete body[index];
        delete geom[index];
    }
    
    void g_vector_save(double vector[1000]);
    void m_vector_save(double vector[1000]);
    void g_mean_vector_save(double vector[1000]);
    void guess_average_save(double vector[1000]);
    
    void Save_Position(btRigidBody *bodypart) {
        btVector3 pos;
        pos = bodypart->getCenterOfMassPosition();
        FILE *dcm;
        char outputFilename[] = "/Users/npowell/Desktop/robotbody/Demos/RagdollDemo/Debug/fits.csv";
        dcm = fopen(outputFilename, "w");
        
        if (dcm == NULL) {
            fprintf(stderr, "Can't open file %s\n",
                    outputFilename);
            exit(1);
        }
        
        fprintf(dcm, "%f,%f,%f\n", pos[0],pos[1], pos[2]);
        fclose(dcm);
    }
    
    //void Save_Categorize(double categorize);
    
    //void Save_Predict(double predict);
    
    //void Save_movement(double predict[1000]);
    
    //void clear_neurons();
    
    //void movement();
    
    //void g_var(double guess_neuron[4]);
    
    //void uncertain();
    
    void Save_Uncertain(double uncertain) {
        
        FILE *unc;
        char outputFilename[] = "/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/uncertainty.csv";
        unc = fopen(outputFilename, "w");
        
        if (unc == NULL) {
            fprintf(stderr, "Can't open file %s\n",
                    outputFilename);
            exit(1);
        }
        
        fprintf(unc, "%f\n", uncertain);
        fclose(unc);
    }
    
    //void update_ANN();
    
    void update_ANN();
    
    void Save_Categorize(double categorize);
    
    void Save_Predict(double predict);
    
    void Save_movement(double predict[1000]);
    
    void clear_neurons();
    
    void movement1(double Prop_current[4], double Prop_previous[4]);
    
    void g_var(double guess_neuron[4]);
    
    void gvar_and_mvect(double vector1[1000], double vector2[1000]);
    
    void uncertain();
    
    void categorizer();
    
    
    void DestroyHinge( int index ) {
        m_dynamicsWorld->removeConstraint( joints[index] );
        delete joints[index];
    }
    
    btVector3 PointWorldToLocal(int index, btVector3 &p) {
        btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
        return local1 * p;
    }
    
    btVector3 AxisWorldToLocal(int index, btVector3 &a) {
        btTransform local1 = body[index]->getCenterOfMassTransform().inverse();
        btVector3 zero(0,0,0);
        local1.setOrigin(zero);
        return local1 * a;
    }
    
    double CalculateVision(int rays, double radius, double spread, btVector3 src, double azi, double zen);
    
    
    void ActuateJoint(int jointIndex, double desiredAngle,
                      double jointOffset, double timeStep);
    
    void ActuateJoint2(int jointIndex, double desiredAngle,
                       double timeStep);
    
    
};





#endif


