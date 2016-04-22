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

    bool oneStep;

    int bodyLookup[4];

    bool pause;

    double offsets[8];

    //    // need to do some collision masking
    //    // and I'm going to ignore when body parts collide with one another
    //    // reference:
    //    // http://bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Filtering
    //    #define BIT(x) (1<<(x))
    //    enum collisiontypes {
    //        COL_NOTHING = 0, // 00000000
    //        COL_LOWER_LEG = BIT(0), // 00000001
    //        COL_UPPER_LEG = BIT(1), // 00000010
    //        COL_BODY = BIT(2), // 00000011
    //        COL_LAND = BIT(6), //00000111
    //    };
    //
    //    int nothingCollidesWith = COL_NOTHING;
    //    int lowerLegCollidesWith = COL_LOWER_LEG;
    //    int upperLegCollidesWith = COL_UPPER_LEG;
    //    int bodyCollidesWith = COL_BODY;
    //    int landCollidesWith = COL_LAND;

    int IDs[6];

public:

    int touches[10];

    double X, Y, Z, RADIUS, LENGTH, SIZE;

    double position_size[1][6];

    double size;

    double categorize;

    double predict;
    double Prop_current;
    double Prop_previous;
    double P_hat_current;
    double P_hat_previous;
    double g_hat;

    btVector3 touchPoints[10];   //This should be proprieceptive somehow

    double hidden_Neurons[4];

    double P_hat[4]; //predictive neurons

    double Prop[4];     //Proprieceptive Neurons

    double motor_neurons[4];
    double ms;

    double guess_neuron[4];

    double weights[4][13];

    float randomNum;

    long timeStep;

    long timeStepExit;

    void initPhysics();

    void exitPhysics();

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

    void CreateBox( int index, double x, double y, double z, double length, double height, double width);

    // void CreateCylinder( int index, double x, double y, double z, double xv, double yv, double zv, char orientation);
    void CreateCylinder( int index, double x, double y, double z, double r, double len, char orientation);

    void CreateHinge(int index, int body1, int body2, double x, double y, double z, double ax, double ay, double az, double theta1, double theta2);

    void DeleteObject( int index ) {
        m_dynamicsWorld->removeRigidBody( body[index] );
        delete body[index];
        delete geom[index];
    }

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

    void Save_Categorize(double categorize) {

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

    void Save_Predict(double predict) {

        FILE *pre;
        char outputFilename[] = "/Users/np/Desktop/Bullet/bullet_make/Demos/RagdollDemo/Debug/predict.csv";
        pre = fopen(outputFilename, "w");

        if (pre == NULL) {
            fprintf(stderr, "Can't open file %s\n",
                    outputFilename);
            exit(1);
        }

        fprintf(pre, "%f\n", predict);
        fclose(pre);
    }

    void clear_neurons() {
        for (int i=0; i <4; i++) {
            Prop[i] = 0.0;
            hidden_Neurons[i] = 0.0;
            P_hat[i] = 0.0;
            guess_neuron[i] = 0.0;
        }
    }

    void predicter() {
        predict = predict + (pow((Prop_current - P_hat_previous), 2.0)/(1000.0 - 1.0));
        //printf("%f", predict);
    }

    void update_ANN() {

        clear_neurons();

        Prop_current = 0.0;
        Prop_previous = 0.0;
        P_hat_current = 0.0;
        P_hat_previous = 0.0;

        for (int i = 0; i < 4; ++i){
            Prop[i] = (joints[i]->getHingeAngle());
            Prop[i] = tanh(Prop[i]);
            Prop_previous = Prop_current;
            Prop_current = Prop[i];
        }


    for (int h=0; h<4; h++) {
        hidden_Neurons[h] = 0.0;
        for (int p=0; p<4; p++){
            hidden_Neurons[h] = hidden_Neurons[h] + (weights[p][h] * Prop[p]) ;
            }
        hidden_Neurons[h] = tanh(hidden_Neurons[h]);
        }

    /*
    for (int h=0; h<4; h++){
        P_hat[h] = (hidden_Neurons[h] * weights[h][h+4]);
        P_hat[h] = tanh(P_hat[h]);
        P_hat_previous = P_hat_current;
        P_hat_current = P_hat[h];
    }
     */

        g_hat = 0.0;
    for (int i=0; i<4; i++) {
        for (int p=0; p<4; p++) {
        guess_neuron[i] = guess_neuron[i] + (hidden_Neurons[p] * weights[i][p+8]);
        }
        guess_neuron[i] = tanh(guess_neuron[i]);

        g_hat = g_hat + guess_neuron[i];     //mean of guess neurons
        }



    for (int i=0; i<4; i++) {
        //printf("%f", Prop[i]);
        //printf("%f", hidden_Neurons[i]);
        //printf("%f", P_hat[i]);
        //printf("%f", guess_neuron[i]);
    }

    for (int h=0; h<4; h++){
        for (int p=0; p<4; p++) {
        motor_neurons[h] = motor_neurons[h] + (hidden_Neurons[h] * weights[h][8+p]);
        }
        motor_neurons[h] = tanh(motor_neurons[h]);
        motor_neurons[h] = motor_neurons[h]*(90);
        ActuateJoint2(h, motor_neurons[h], ms / 900000.f);
        //printf("%f", motor_neurons[h]);

        }

        predicter();
    }

    void categorizer(double SIZE, double g_hat) {
        categorize = categorize + (pow((SIZE - g_hat), 2.0)) / (1000.0);
        //printf("%f", categorize);
    }


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



    void ActuateJoint(int jointIndex, double desiredAngle,
                      double jointOffset, double timeStep);

    void ActuateJoint2(int jointIndex, double desiredAngle,
                       double timeStep);


};





#endif
