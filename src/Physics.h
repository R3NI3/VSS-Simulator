/*The MIT License (MIT)

Copyright (c) 2016 Lucas Borsatto Simão

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
*/

#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "Header.h"
#include "RobotStrategy.h"
#include "GLDebugDrawer.h"
#include "RobotPhysics.h"

const Color clrTeams[] = {Color(1.0,1.0,0),Color(0,0,1)};
const Color clrPlayers[] = {Color(1,0,0),Color(0,1,0),Color(1,0.5,0.5)};
const float SIMULATION_TIME_STEP = (1.f/60.f);

class Physics{
private:
	int numTeams;
	int numRobotsTeam;
	int initAgents;
	int initBall;

	btDynamicsWorld* world;
	btDispatcher* dispatcher;
	btCollisionConfiguration* collisionConfig;
	btBroadphaseInterface* broadphase;
	btConstraintSolver* solver;
	GLDebugDrawer* glDebugDrawer;

	vector<BulletObject*> bodies;
	vector<RobotPhysics*> genRobots;

   void registBodies();
	void setupBodiesProp();

	btRigidBody* addGenericBody(btCollisionShape* shape,string name, Color clr, btVector3 pos, float mass, btVector3 rotation = btVector3(0,0,0));
    static bool callBackHitFunc(btManifoldPoint& cp,const btCollisionObjectWrapper* obj1,int id1,int index1,const btCollisionObjectWrapper* obj2,int id2,int index2);
public:
	Physics(int numTeams, int initAgents, int initBall);
	~Physics();
	void deleteWorldObj();

	btRigidBody* addFloor();
	btRigidBody* addBall(float rad,btVector3 pos,float mass);
	btRigidBody* addWall(Color clr, btVector3 pos,float width, float height, float depth, float mass);
	btRigidBody* addCorner(Color clr, btVector3 pos,float width, float height,btVector3 rotation = btVector3(0,0,0));
	RobotPhysics* addRobot(Color clr, btVector3 pos,btVector3 rotation,float sizeRobot, float mass,Color clrPlayer,Color clrTeam, int id);

	void stepSimulation(float time,float subStep, float timeStep);

	void startDebug();
	void setDebugWorld(int debugMode);

	 //void resetRobotPositions();
	 void init_positions();
    void init_default_positions();
    void init_random_positions();
    void init_base_positions();
	 void init_one_agent_random();
    void init_goalkeeper_train();
    void init_penalty_team_1();
    void init_penalty_team_2();

    void init_ball_center();
    void init_rand_slow();
    void init_towards_left_goal();
    void init_towards_right_goal();
	 void init_towards_rand_left_right_goal();

    void test_penalty();

    int getNumTeams(){ return numTeams; }
	btDynamicsWorld* getWorldPhysics() { return world; }
	btVector3 getBallPosition();
	btVector3 getBallVelocity();
	void setBallPosition(btVector3);
	void setBallVelocity(btVector3);

	vector<BulletObject*> getAllBtObj() { return bodies; }
	vector<RobotPhysics*> getAllRobots() { return genRobots; }
	vector<BulletObject*> getAllBtRobots();
	btVector3 getRobotPosition(RobotPhysics* robot);
	void setRobotsPosition(vector<btVector3>);
	void setRobotsPosition(vector<btVector3>, vector<btVector3>);
};

#endif
