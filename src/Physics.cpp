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

#include "Physics.h"
#include <sstream>

Physics::Physics(int numTeams, int initAgents, int initBall){
    this->numTeams = numTeams;
    this->numRobotsTeam = NUM_ROBOTS_TEAM;
    this->initAgents = initAgents;
    this->initBall = initBall;

    collisionConfig = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfig);
    broadphase = new btDbvtBroadphase();
    solver = new btSequentialImpulseConstraintSolver();
    world = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfig);
    world->setGravity(btVector3(0,-9.81*SCALE_WORLD,0));

    glDebugDrawer = new GLDebugDrawer();
    world->setDebugDrawer(glDebugDrawer);
    gContactAddedCallback = callBackHitFunc;

    registBodies();
    this->init_positions();
}

Physics::~Physics(){
    deleteWorldObj();

    if(collisionConfig) delete collisionConfig;
    if(dispatcher) delete dispatcher;
    if(broadphase) delete broadphase;
    if(solver) delete solver;
    if(world) delete world;
}

void Physics::deleteWorldObj(){
    for(int i = 0; i<bodies.size();i++){
        delete bodies[i];
    }

    for(int i = 0;i<genRobots.size();i++){
        delete genRobots[i];
    }
}

bool check_dist(vector <btVector3> vec, btVector3 t){
    bool ret = true;
    int sqr_dist;

    for(vector<btVector3>::iterator it = vec.begin(); it != vec.end(); ++it){
        sqr_dist = pow(it->getX()-t.getX(),2) + pow(it->getZ()-t.getZ(),2);
        if (sqr_dist < 144){
            ret = false;
            break;
        }
    } 

    return ret;
}

void Physics::test_penalty() {
    vector<btVector3> robots;
    vector<btVector3> angles;

//team 1
//    //offenders
//    robots.push_back(btVector3(15,4,SIZE_DEPTH/2.0-10));
//    robots.push_back(btVector3(15,4,SIZE_DEPTH/2.0+10));
//
//    //others
//    robots.push_back(btVector3(80,4,78));
//
//    //goalkeeper
//    robots.push_back(btVector3(160,4,SIZE_DEPTH/2.0));
//    angles.push_back(btVector3(0,90,0));
//
//    robots.push_back(btVector3(80,4,52));
//    angles.push_back(btVector3(0,90,0));
//
//    robots.push_back(btVector3(80,4,104));
//    angles.push_back(btVector3(0,90,0));
//
//    setBallPosition(btVector3(11, 2.0, SIZE_DEPTH/2.0+15));

//Team 2
    //others
    robots.push_back(btVector3(80,4,78));

    //goalkeeper
    robots.push_back(btVector3(160,4,SIZE_DEPTH/2.0));
    angles.push_back(btVector3(0,90,0));

    robots.push_back(btVector3(80,4,52));
    angles.push_back(btVector3(0,90,0));

    robots.push_back(btVector3(80,4,104));
    angles.push_back(btVector3(0,90,0));

    //offenders
    robots.push_back(btVector3(150,4,SIZE_DEPTH/2.0-10));
    robots.push_back(btVector3(150,4,SIZE_DEPTH/2.0+10));

    setBallPosition(btVector3(159, 2.0, SIZE_DEPTH/2.0+15));

    setRobotsPosition(robots);
}


void Physics::init_penalty_team_1() {
    vector<btVector3> robots;
    vector<btVector3> angles;

    //striker
    robots.push_back(btVector3(114-2+rand()%4,5,SIZE_DEPTH/2.0-2+rand()%4));
    angles.push_back(btVector3(0,90-5+rand()%10,0));

    robots.push_back(btVector3(80,5,26));
    angles.push_back(btVector3(0,rand()%360,0));

    robots.push_back(btVector3(80,5,78));
    angles.push_back(btVector3(0,rand()%360,0));

    robots.push_back(btVector3(80,5,52));
    angles.push_back(btVector3(0,rand()%360,0));

    robots.push_back(btVector3(80,5,104));
    angles.push_back(btVector3(0,rand()%360,0));

    //goalkeeper
    robots.push_back(btVector3(160-2+rand()%4,5,SIZE_DEPTH/2.0-2+rand()%4));
    if (rand()%2==0)
        angles.push_back(btVector3(0,90-5+rand()%10,0));
    else
        angles.push_back(btVector3(0,0-5+rand()%10,0));

    setBallPosition(btVector3(122.5-1+rand()%2, 2.0, SIZE_DEPTH/2.0-1+rand()%2));
    setRobotsPosition(robots, angles);
}

void Physics::init_penalty_team_2() {
    vector<btVector3> robots;
    vector<btVector3> angles;

    robots.push_back(btVector3(90,5,52));
    angles.push_back(btVector3(0,rand()%360,0));

    robots.push_back(btVector3(90,5,104));
    angles.push_back(btVector3(0,rand()%360,0));

//goalkeeper
    robots.push_back(btVector3(10-2+rand()%4,5,SIZE_DEPTH/2.0-2+rand()%4));
    if (rand()%2==0)
        angles.push_back(btVector3(0,90-5+rand()%10,0));
    else
        angles.push_back(btVector3(0,0-5+rand()%10,0));

    //striker
    robots.push_back(btVector3(56-2+rand()%4,5,SIZE_DEPTH/2.0-2+rand()%4));
    angles.push_back(btVector3(0,90-5+rand()%10,0));

    robots.push_back(btVector3(90,5,26));
    angles.push_back(btVector3(0,rand()%360,0));
    robots.push_back(btVector3(90,5,78));
    angles.push_back(btVector3(0,rand()%360,0));

    setBallPosition(btVector3(47.5, 2.0-1+rand()%2, SIZE_DEPTH/2.0-1+rand()%2));
    setRobotsPosition(robots, angles);
}

void Physics::init_goalkeeper_train() {

        vector<btVector3> robots;
        vector<btVector3> angles;
        vector<btVector3> posBall;
        posBall.push_back(btVector3((rand()%150)+10, 0, (rand()%110)+10));
        int x1, z1, ang1;

        //others

        for(int i = 0;i < numRobotsTeam-1;i++){
            x1 = (rand()%130)+20;
            z1 = (rand()%110)+10;
            ang1 = (rand()%360);

            btVector3 pos1 = btVector3(x1, 5, z1);
            angles.push_back(btVector3(0,ang1,0));

            while (!(check_dist(robots, pos1) && check_dist(posBall, pos1))){
                x1 = (rand()%130)+20;
                z1 = (rand()%110)+10;
                pos1 = btVector3(x1, 5, z1);
            }
            robots.push_back(pos1);
        }

        //goalkeeper left
        robots.push_back(btVector3(10-2+rand()%4,5, SIZE_DEPTH/2.0-12+rand()%24));
        if (rand()%2==0)
            angles.push_back(btVector3(0,90-5+rand()%10,0));
        else
            angles.push_back(btVector3(0,0-5+rand()%10,0));

         //others

        for(int i = 0;i < numRobotsTeam-1;i++){
            x1 = (rand()%130)+20;
            z1 = (rand()%110)+10;
            ang1 = (rand()%360);

            btVector3 pos1 = btVector3(x1, 5, z1);
            angles.push_back(btVector3(0,ang1,0));

            while (!(check_dist(robots, pos1) && check_dist(posBall, pos1))){
                x1 = (rand()%130)+20;
                z1 = (rand()%110)+10;
                pos1 = btVector3(x1, 5, z1);
            }
            robots.push_back(pos1);
        }

        //goalkeeper right
        robots.push_back(btVector3(160-2+rand()%4,5,SIZE_DEPTH/2.0-12+rand()%24));
        if (rand()%2==0)
            angles.push_back(btVector3(0,90-5+rand()%10,0));
        else
            angles.push_back(btVector3(0,0-5+rand()%10,0));

        setRobotsPosition(robots, angles);

        setBallPosition(posBall[0]);
        setBallVelocity(btVector3((-90*(rand()%100))/100.0-10, 0, (2*(rand()%100))/100.0-1));
}

void Physics::init_one_agent_random() {

	vector<btVector3> robots;
	vector<btVector3> angles;
	int x1, z1, ang1;

	x1 = (rand()%130)+20;
	z1 = (rand()%110)+10;
	ang1 = rand()%360;

	setBallPosition(btVector3((rand()%150)+10, 0, (rand()%100)+10));
   setBallVelocity(btVector3((2*(rand()%100))/100.0-1, 0, (8*(rand()%100))/100.0-4));

	btVector3 pos1 = btVector3(x1, 5, z1);
	robots.push_back(pos1);
	angles.push_back(btVector3(0,ang1,0));
	
	for(int i = 1;i < 2*numRobotsTeam;i++){
		robots.push_back(btVector3(20+10*i, 5, -20));
		angles.push_back(btVector3(0,0,0));
	}

   setRobotsPosition(robots, angles);
}


void Physics::init_default_positions() {

     vector<btVector3> robots;

     robots.push_back(btVector3(55,4,45));
     robots.push_back(btVector3(35,4,30));
     robots.push_back(btVector3(15,4,SIZE_DEPTH- 55));
     robots.push_back(btVector3(SIZE_WIDTH-55,4,85));
     robots.push_back(btVector3(SIZE_WIDTH-25,4,SIZE_DEPTH - SIZE_DEPTH/2.5 + 20));
     robots.push_back(btVector3(SIZE_WIDTH-15,4,55));

     setBallPosition(btVector3( (SIZE_WIDTH/2.0)+10 , 2.0, SIZE_DEPTH/2.0));
     setRobotsPosition(robots);
}

void Physics::init_base_positions() {

     vector<btVector3> rbt1, rbt2;
     vector<btVector3> angles;
     vector<btVector3> posBall;
     posBall.push_back(btVector3((rand()%130)+20, 0, (rand()%100)+10));
     int x1, x2, z1, z2, ang1, ang2;

     for(int i = 0;i < numRobotsTeam;i++){
			if (i==0) {
				x1 = (rand()%40)+110;
				x2 = 170-((rand()%40)+110);
			}
			else if (i==1){
				x1 = (rand()%75)+30;
				x2 = 170-((rand()%75)+30);
			}
			else if (i==2) {
				x1 = (rand()%10)+20;
				x2 = 170-((rand()%10)+20);
			}

         z1 = (rand()%100)+15;
         z2 = (rand()%100)+15;
         ang1 = rand()%360;
         ang2 = rand()%360;

         btVector3 pos1 = btVector3(x1, 5, z1);
         btVector3 pos2 = btVector3(x2, 5, z2);
      
		   while (!(check_dist(rbt1, pos1) && check_dist(rbt2, pos1) && check_dist(posBall, pos1))){
					if (i==0) {
						x1 = (rand()%40)+110;
					}
					else if (i==1){
						x1 = (rand()%75)+30;
					}
					else if (i==2) {
						x1 = (rand()%10)+20;
					}
					z1 = (rand()%100)+15;
					pos1 = btVector3(x1, 5, z1);
         }

         rbt1.push_back(pos1);
         angles.push_back(btVector3(0,ang1,0));

         while (!(check_dist(rbt1, pos2) && check_dist(rbt2, pos1) && check_dist(posBall, pos2))){
				if (i==0) {
					x2 = 170-((rand()%40)+110);
				}
				else if (i==1){
					x2 = 170-((rand()%75)+30);
				}
				else if (i==2) {
					x2 = 170-((rand()%10)+20);
				}

				z2 = (rand()%100)+15;
				pos2 = btVector3(x2, 5, z2);
         }

         rbt2.push_back(pos2);
         angles.push_back(btVector3(0,ang2,0));
     }

	  rbt1.insert(std::end(rbt1), std::begin(rbt2), std::end(rbt2));
     setRobotsPosition(rbt1, angles);

     setBallPosition(posBall[0]);
     setBallVelocity(btVector3((2*(rand()%100))/100.0-1, 0, (8*(rand()%100))/100.0-4));
}

void Physics::init_random_positions() {

     vector<btVector3> robots;
     vector<btVector3> angles;
     vector<btVector3> posBall;
     posBall.push_back(btVector3((rand()%130)+20, 0, (rand()%100)+10));
     int x1, x2, z1, z2, ang1, ang2;

     for(int i = 0;i < numRobotsTeam;i++){
         x1 = (rand()%130)+20;
         x2 = (rand()%130)+20;
         ang1 = rand()%360;
         z1 = (rand()%110)+10;
         z2 = (rand()%110)+10;
         ang2 = rand()%360;

         btVector3 pos1 = btVector3(x1, 5, z1);
         btVector3 pos2 = btVector3(x2, 5, z2);

         while (!(check_dist(robots, pos1) && check_dist(posBall, pos1))){
             x1 = (rand()%130)+20;
             z1 = (rand()%110)+10;
             pos1 = btVector3(x1, 5, z1);
         }
         robots.push_back(pos1);
         angles.push_back(btVector3(0,ang1,0));

         while (!(check_dist(robots, pos2) && check_dist(posBall, pos2))){
             x2 = (rand()%130)+20;
             z2 = (rand()%110)+10;
             pos2 = btVector3(x2, 5, z2);
         }
         robots.push_back(pos2);
         angles.push_back(btVector3(0,ang2,0));
     }

     setRobotsPosition(robots, angles);

     setBallPosition(posBall[0]);
     setBallVelocity(btVector3((2*(rand()%100))/100.0-1, 0, (8*(rand()%100))/100.0-4));
}

void Physics::init_ball_center() {

	setBallPosition(btVector3( (SIZE_WIDTH/2.0)+10 , 2.0, SIZE_DEPTH/2.0));
	setBallVelocity(btVector3(0, 0, 0));
}

void Physics::init_rand_slow() {

	setBallPosition(btVector3((rand()%130)+20, 2.0, (rand()%100)+10));
	setBallVelocity(btVector3((2*(rand()%100))/100.0-1, 0, (8*(rand()%100))/100.0-4));
}

void Physics::init_towards_left_goal() {

	int x = (rand()%130)+20;
	int z = (rand()%100)+10;
	setBallPosition(btVector3(x, 0, z));

   setBallVelocity(btVector3((-90*(rand()%100))/100.0-10, 2.0, -(z-65)/2));
}

void Physics::init_towards_right_goal() {

	int x = (rand()%130)+20;
	int z = (rand()%100)+10;
	setBallPosition(btVector3(x, 0, z));

   setBallVelocity(btVector3((90*(rand()%100))/100.0+10, 2.0, -(z-65)/2));
}

void Physics::init_towards_rand_left_right_goal() {
	if (rand()%1000 > 500)
		init_towards_left_goal();
   else
		init_towards_right_goal();
}

void Physics::init_positions() {
	//printf("init agents: %d init ball: %d", this->initAgents, this->initBall);
	switch (this->initAgents) {
		case 1: this->init_random_positions(); break;
		case 2: this->init_base_positions(); break;
		case 3: this->init_one_agent_random(); break;
		case 4: this->init_goalkeeper_train(); break;
		case 5: this->init_penalty_team_1(); break;
		case 6: this->init_penalty_team_2(); break;
		case 0:
		default: this->init_default_positions(); break;
   }

	switch (this->initBall) {
		case 0: this->init_ball_center(); break;
		case 1: this->init_rand_slow(); break;
		case 2: this->init_towards_left_goal(); break;
		case 3: this->init_towards_right_goal(); break;
		case 4: this->init_towards_rand_left_right_goal(); break;
	 }
}


void Physics::registBodies(){
    addFloor();
    time_t t;
    srand((unsigned) time(&t));
    vector <btVector3> posTeam1;
    vector <btVector3> posTeam2;
    vector <btVector3> posBall;
    int x1,x2,z1,z2;

    addBall(2.5, btVector3(85, 0, 65), 0.08); //center
    posTeam1 = vector <btVector3> {btVector3(25,5,SIZE_DEPTH- 55),btVector3(35,5,30),btVector3(55,5,45)};
    posTeam2 = vector <btVector3> {btVector3(SIZE_WIDTH-15,5,55),btVector3(SIZE_WIDTH-25,5,SIZE_DEPTH - SIZE_DEPTH/2.5 + 20),btVector3(SIZE_WIDTH-55,5,85)};

    //Create robots here
    //Team 1
    for(int i = 0;i < numRobotsTeam;i++){
        if(numTeams >= 1){
            addRobot(Color(0.3,0.3,0.3),posTeam1[i],btVector3(0,90,0),8,0.25,clrPlayers[i],clrTeams[0], i);
        }
    }

    for(int i = 0;i < numRobotsTeam;i++){
        if(numTeams == 2){
            addRobot(Color(0.3,0.3,0.3),posTeam2[i],btVector3(0,-100,0),8,0.25,clrPlayers[i],clrTeams[1], numRobotsTeam+i);
        }
    }

    // PAREDE DE CIMA
    addWall(Color(0,0,0), btVector3((SIZE_WIDTH/2.0) + GOAL_WIDTH, 0, 0), SIZE_WIDTH, 15, 2.5, 0);
    // PAREDE DE BAIXO
    addWall(Color(0,0,0), btVector3((SIZE_WIDTH/2.0) + GOAL_WIDTH, 0, SIZE_DEPTH), SIZE_WIDTH, 15, 2.5, 0);

    // GOL ESQUERDO
    addWall(Color(0,0,0), btVector3(0, 0, SIZE_DEPTH/2.0), 2.5, 15, 40.0, 0);
    // PAREDE DE CIMA DO GOL ESQUERDO
    addWall(Color(0,0,0), btVector3(GOAL_WIDTH, 0, 45/2.0-1.25), 2.5, 15, 45.0, 0);
    // PAREDE DE BAIXO DO GOL ESQUERDO
    addWall(Color(0,0,0), btVector3(GOAL_WIDTH, 0, SIZE_DEPTH-(45/2.0)+1.25 ), 2.5, 15, 45.0, 0);
    // PAREDE DE CIMA DENTRO DO GOL ESQUERDO
    addWall(Color(0,0,0), btVector3((GOAL_WIDTH/2.0), 0, SIZE_DEPTH/2.0-(45/2.0) ), GOAL_WIDTH, 15, 2.5, 0);
    // PAREDE DE BAIXO DENTRO DO GOL ESQUERDO
    addWall(Color(0,0,0), btVector3((GOAL_WIDTH/2.0), 0, SIZE_DEPTH/2.0+(45/2.0) ), GOAL_WIDTH, 15, 2.5, 0);

    // GOL DIREITO
    addWall(Color(0,0,0), btVector3(SIZE_WIDTH + (2.0*GOAL_WIDTH), 0, SIZE_DEPTH/2.0), 2.5, 15, 40.0, 0);
    // PAREDE DE CIMA DO GOL DIREITO
    addWall(Color(0,0,0), btVector3(SIZE_WIDTH + GOAL_WIDTH, 0, (45/2.0)-1.25), 2.5, 15, 45.0, 0);
    // PAREDE DE BAIXO DO GOL DIREITO
    addWall(Color(0,0,0), btVector3(SIZE_WIDTH + GOAL_WIDTH, 0, SIZE_DEPTH-(45/2.0)+1.25 ), 2.5, 15, 45.0, 0);
    // PAREDE DE CIMA DENTRO DO GOL DIREITO
    addWall(Color(0,0,0), btVector3(SIZE_WIDTH + (2.0*GOAL_WIDTH)-(GOAL_WIDTH/2.0), 0, SIZE_DEPTH/2.0-(45/2.0)), GOAL_WIDTH, 15, 2.5, 0);
    // PAREDE DE BAIXO DENTRO DO GOL DIREITO
    addWall(Color(0,0,0), btVector3(SIZE_WIDTH + (2.0*GOAL_WIDTH)-(GOAL_WIDTH/2.0), 0, SIZE_DEPTH/2.0+(45/2.0)), GOAL_WIDTH, 15, 2.5, 0);

    // TRIANGULO SUPERIOR ESQUERDO
    addCorner(Color(0,0,0),btVector3(SIZE_WIDTH + (GOAL_WIDTH), 0, GOAL_WIDTH+1.25), 45, 15, btVector3(0,-45,0));
    // TRIANGULO SUPERIOR DIREITO
    addCorner(Color(0,0,0),btVector3(GOAL_WIDTH, 0, GOAL_WIDTH+1.25), 45, 15, btVector3(0,45,0));
    
    // TRIANGULO INFERIOR ESQUERDO
    addCorner(Color(0,0,0),btVector3(GOAL_WIDTH+1.25, 0, SIZE_DEPTH-GOAL_WIDTH+1.25), 45, 15, btVector3(0,-45,0));
    // TRIANGULO INFERIOR DIREITO
    addCorner(Color(0,0,0),btVector3(SIZE_WIDTH + GOAL_WIDTH-1.25, 0, SIZE_DEPTH-GOAL_WIDTH-1.25), 45, 15, btVector3(0,45,0));
}

/*
void Physics::resetRobotPositions(){
    btVector3 posTeam1[] = {btVector3(15,5,SIZE_DEPTH- 55),btVector3(35,5,30),btVector3(55,5,45)};
    btVector3 posTeam2[] = {btVector3(SIZE_WIDTH-15,5,55),btVector3(SIZE_WIDTH-25,5,SIZE_DEPTH - SIZE_DEPTH/2.5 + 20),btVector3(SIZE_WIDTH-55,5,85)};

//    btVector3 axis = rotation.normalize();
//    btQuaternion quat(axis,rad);
//    t.setRotation(quat);

    for(int i=0;i<genRobots.size()/2;i++){
        btTransform t;
        genRobots[i]->getRigidBody()->getMotionState()->getWorldTransform(t);

        t.setIdentity();
        t.setOrigin(posTeam1[i]);
        
        btMotionState* motion = new btDefaultMotionState(t);
        
        genRobots[i]->getRigidBody()->setMotionState(motion);
        genRobots[i]->getRigidBody()->setLinearVelocity(btVector3(0,0,0));
        genRobots[i]->getRigidBody()->setAngularVelocity(btVector3(0,0,0));
    }

    for(int i=genRobots.size()/2;i<genRobots.size();i++){
        btTransform t;
        genRobots[i]->getRigidBody()->getMotionState()->getWorldTransform(t);

        t.setIdentity();
        t.setOrigin(posTeam2[i-genRobots.size()/2]);
        
        btMotionState* motion = new btDefaultMotionState(t);
        
        genRobots[i]->getRigidBody()->setMotionState(motion);
        genRobots[i]->getRigidBody()->setLinearVelocity(btVector3(0,0,0));
        genRobots[i]->getRigidBody()->setAngularVelocity(btVector3(0,0,0));
    }
}
*/

void Physics::stepSimulation(float timeW,float subStep, float timeStep){
    setupBodiesProp();
    world->stepSimulation(timeW, subStep, timeStep);
}

void Physics::setupBodiesProp(){
    for(int i = 0; i < genRobots.size(); i++){
        bodies.at(i)->hitRobot = false;
        bodies.at(i)->hit = false;
    }
}

vector<BulletObject*> Physics::getAllBtRobots(){
    vector<BulletObject*> listRobots;
    string prefix = "robot";
    for(int i = 0; i < bodies.size();i++) {
        string name = bodies.at(i)->name;
        if(!name.compare(0, prefix.size(), prefix)) {
            listRobots.push_back(bodies.at(i));
        }
    }

    return listRobots;
}

bool Physics::callBackHitFunc(btManifoldPoint& cp,const btCollisionObjectWrapper* obj1,int id1,int index1,const btCollisionObjectWrapper* obj2,int id2,int index2){
    string prefix = "robot";

    const btCollisionObjectWrapper* wrappers[] = {obj1, obj2};

    vector<BulletObject*> vecObj;

    int contAgent = 0;
    for(int i = 0; i < 2; i++){
        BulletObject* btObj = (BulletObject*)wrappers[i]->getCollisionObject()->getUserPointer();

        vecObj.push_back(btObj);

        string name = btObj->name;
        if (!name.compare(0, prefix.size(), prefix) || name == "ball")
            btObj->hit = true;

        if(!name.compare(0, prefix.size(), prefix)){
            contAgent++;
        }
    }

    if(contAgent == 2){
        vecObj.at(0)->hitRobot = true;
        vecObj.at(1)->hitRobot = true;
    }
        
    return false;
}

btVector3 Physics::getBallPosition(){
	btVector3 ballPos;
	for(int i=0;i<bodies.size();i++){
		if(bodies[i]->name.compare("ball") == 0){
			btTransform t;
			bodies[i]->body->getMotionState()->getWorldTransform(t);
            ballPos = t.getOrigin();
			break;
		}
	}
	return ballPos;
}

btVector3 Physics::getRobotPosition(RobotPhysics* robot){
    btTransform  transTemp;
    robot->getRigidBody()->getMotionState()->getWorldTransform(transTemp);
    return transTemp.getOrigin();
}

btVector3 Physics::getBallVelocity(){
    btVector3 ballVel;
	for(int i=0;i<bodies.size();i++){
		if(bodies[i]->name.compare("ball") == 0){
			//btTransform t;
			//bodies[i]->body->getMotionState()->getWorldTransform(t);
            ballVel = bodies[i]->body->getLinearVelocity();
			break;
		}
	}
    //cout << ballVel.getX() << ", " << ballVel.getY() << ", " << ballVel.getZ() << endl;
	return ballVel;
}

void Physics::setBallPosition(btVector3 newPos){
    for(int i=0;i<bodies.size();i++){
        if(bodies[i]->name.compare("ball") == 0){
            btTransform t;
            bodies[i]->body->getMotionState()->getWorldTransform(t);

            t.setIdentity();
            t.setOrigin(newPos);
            
            btMotionState* motion = new btDefaultMotionState(t);
            
            bodies[i]->body->setMotionState(motion);
            bodies[i]->body->setLinearVelocity(btVector3(0,0,0));
            bodies[i]->body->setAngularVelocity(btVector3(0,0,0));

            break;
        }
    }
}

void Physics::setBallVelocity(btVector3 newVel){
    for(int i=0;i<bodies.size();i++){
        if(bodies[i]->name.compare("ball") == 0){
            btTransform t;

            bodies[i]->body->setLinearVelocity(newVel);

            break;
        }
    }
}

void Physics::setRobotsPosition(vector<btVector3> new_poses){
    for(int i=0;i<genRobots.size();i++){
        btTransform t;
        genRobots[i]->getRigidBody()->getMotionState()->getWorldTransform(t);

        t.setIdentity();
        t.setOrigin(new_poses[i]);
        
        btMotionState* motion = new btDefaultMotionState(t);
        
        genRobots[i]->getRigidBody()->setMotionState(motion);
        genRobots[i]->getRigidBody()->setLinearVelocity(btVector3(0,0,0));
        genRobots[i]->getRigidBody()->setAngularVelocity(btVector3(0,0,0));
    }
}

void Physics::setRobotsPosition(vector<btVector3> new_poses, vector<btVector3> rotations){
    for(int i=0;i<genRobots.size();i++){
        btTransform t;
        genRobots[i]->getRigidBody()->getMotionState()->getWorldTransform(t);

        t.setIdentity();
        t.setOrigin(new_poses[i]);

        btVector3 rotation = rotations[i];
        if (rotation.length()!=0) {
            rotation *= PI/180;
            float rad = rotation.length();
            btVector3 axis = rotation.normalize();
            btQuaternion quat(axis,rad);
            t.setRotation(quat);
        }

        btMotionState* motion = new btDefaultMotionState(t);

        genRobots[i]->getRigidBody()->setMotionState(motion);
        genRobots[i]->getRigidBody()->setLinearVelocity(btVector3(0,0,0));
        genRobots[i]->getRigidBody()->setAngularVelocity(btVector3(0,0,0));
    }
}

void Physics::startDebug(){
    world->debugDrawWorld();
}

void Physics::setDebugWorld(int debugMode){
    vector<int> debugDrawMode;
    ((GLDebugDrawer*)world-> getDebugDrawer())->setDrawScenarioMode(true);
    switch (debugMode){
        case 0:{
            debugDrawMode.push_back(btIDebugDraw::DBG_NoDebug);
            world->getDebugDrawer()->setDebugMode(debugDrawMode);
        }break;
        case 1:{
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawLocalProperties);
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawWireframe);
            world->getDebugDrawer()->setDebugMode(debugDrawMode);
            ((GLDebugDrawer*)world-> getDebugDrawer())->setDrawScenarioMode(false);
        }break;
        case 2:{
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawWireframe);
            debugDrawMode.push_back(btIDebugDraw::DBG_DrawLocalProperties);
            world-> getDebugDrawer()->setDebugMode(debugDrawMode);
        }break;
    }
}


btRigidBody* Physics::addFloor(){
	string name = "floor";
    btStaticPlaneShape* plane = new btStaticPlaneShape(btVector3(0,1,0),0);
    btRigidBody* body = addGenericBody(plane,name,Color(0.0,0.0,0.0),btVector3(1.0,0.0,0.0),0);
    return body;
}

btRigidBody* Physics::addBall(float rad,btVector3 pos,float mass)
{
    string name = "ball";
    btSphereShape* ball = new btSphereShape(rad);
    btRigidBody* body = addGenericBody(ball,name,Color(1.0,0.0,0.0), pos, mass);
    return body;
}

btRigidBody* Physics::addWall(Color clr, btVector3 pos, float width, float height, float depth, float mass){
    string name = "wall";
    btBoxShape* wall = new btBoxShape(btVector3(width/2.0,height/2.0,depth/2.0));
    btRigidBody* body = addGenericBody(wall,name,clr,pos,mass);
    return body;
}

btRigidBody* Physics::addCorner(Color clr, btVector3 pos,float width, float height,btVector3 rotation){
    float mass = 0.f;
    float depth = 0.01f;
    btVector3 rotRad = rotation*PI/180;
    string name = "corner";
    btBoxShape* corner = new btBoxShape(btVector3(width/2.0,height/2.0,depth/2.0));
    btRigidBody* body = addGenericBody(corner,name,clr,pos,mass,rotRad);
    return body;
}

RobotPhysics* Physics::addRobot(Color clr, btVector3 pos, btVector3 rotation,float sizeRobot, float mass,Color colorPlayer,Color colorTeam, int id){
    btBoxShape* modelShape = new btBoxShape(btVector3(sizeRobot/2,sizeRobot/2,sizeRobot/2));
    btCompoundShape* compound = new btCompoundShape();

    btTransform localTrans;
    localTrans.setIdentity();
    localTrans.setOrigin(btVector3(0.0,5.0,0));

    compound->addChildShape(localTrans,modelShape);

    btTransform transRobot;
    transRobot.setIdentity();
    transRobot.setOrigin(btVector3(pos.getX(),pos.getY(),pos.getZ()));
    if(rotation.length() != 0){
        rotation *= PI/180;
        float rad = rotation.length();
        btVector3 axis = rotation.normalize();
        btQuaternion quat(axis,rad);
        transRobot.setRotation(quat);
    }

    btVector3 localInertia(0,0,0);
    compound->calculateLocalInertia(mass,localInertia);

    btMotionState* robotMotState = new btDefaultMotionState(transRobot);
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,robotMotState,compound,localInertia);
    btRigidBody* bdRobot = new btRigidBody(cInfo);
    bdRobot->setCollisionFlags(bdRobot->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

    bdRobot -> setLinearVelocity(btVector3(0,0,0));
    bdRobot -> setAngularVelocity(btVector3(0,0,0));

    world->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(
        bdRobot-> getBroadphaseHandle(),
        world -> getDispatcher()
    );

    //bdRobot->setIdDebug(1);

    stringstream st;
    st << "robot-";
    st << id;
    string nameRobot = st.str();

    bodies.push_back(new BulletObject(bdRobot,nameRobot,clr));
    bodies[bodies.size()-1]->halfExt = modelShape->getHalfExtentsWithMargin();
    bdRobot->setUserPointer(bodies[bodies.size()-1]);

    world->addRigidBody (bdRobot);

    RobotPhysics* localRobot = new RobotPhysics(pos,0.2,bdRobot,colorPlayer,colorTeam);
    localRobot->buildRobot(world);

    world->addVehicle(localRobot-> getRaycast());

    genRobots.push_back(localRobot);
    return localRobot;
}

btRigidBody* Physics::addGenericBody(btCollisionShape* shape,string name,Color clr, btVector3 pos, float mass,btVector3 rotation){

	btTransform t;
    t.setIdentity();
    t.setOrigin(btVector3(pos.getX(),pos.getY(),pos.getZ()));

    if(rotation.length() != 0){
        float rad = rotation.length();
        btVector3 axis = rotation.normalize();
        btQuaternion quat(axis,rad);
        t.setRotation(quat);
    }

    btVector3 inertia(0,0,0);
    if(mass!=0.0)
        shape->calculateLocalInertia(mass,inertia);

    btMotionState* motion=new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(mass,motion,shape,inertia);
    btRigidBody* body=new btRigidBody(info);

    bodies.push_back(new BulletObject(body,name,clr));
    body->setUserPointer(bodies[bodies.size()-1]);
    world->addRigidBody(body);
    return body;
}
