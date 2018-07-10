#include "Arbiter.h"
#include <sys/time.h>

Arbiter::Arbiter(){
	refresh = false;
	minutes = 0;
    sysLastTime = -1;
    simLastTime = -1;
}

void Arbiter::allocPhysics(Physics *physics){
	this->physics = physics;
}

void Arbiter::allocReport(Report *report){
	this->report = report;
}

long int Arbiter::sysTimeMS() {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

int Arbiter::checkWorld(){
	int situation = NONE;
	btVector3 ball = physics->getBallPosition();

	if(ball.getX() > 160){
		refresh = true;
		situation = GOAL_TEAM1;
		report->total_of_goals_team[0]++;
		cerr << "---Goal RIGHT---" << endl;
		position_objects_after_goal_team_1();
	}else
	if(ball.getX() < 10){
		refresh = true;
		situation = GOAL_TEAM2;
		report->total_of_goals_team[1]++;
		cerr << "---Goal LEFT---" << endl;
		position_objects_after_goal_team_2();
	}

	vector<RobotPhysics*> listRobots = physics->getAllRobots();

    if (sysLastTime<0) {
        sysLastTime = sysTimeMS();
        simLastTime = checkTimeMs();
    }
	//! A cada minuto mostra o tempo, isto é, 1 min, 2 min, n min.
	if(report->qtd_of_steps/3600 >= minutes+1){
        //system time:  
        long int sysNow = sysTimeMS();
        long int sysDt = sysNow - sysLastTime;
        sysLastTime = sysNow;

        //simulation time:
        long int simNow = checkTimeMs();
        long int simDt = simNow - simLastTime;
        simLastTime = simNow;

        float rate = 0;
        if (sysDt>0)
            rate = (simDt/(float)sysDt);    

		//! É necessário verificar dessa maneira, pois como qtd_of_steps é um variavel que roda na simulação de física
		//! e a simulação física trabalha em uma frequência maior que o resto do programa, nem sempre o loop no "valor 3500"
		//! caia na verificação. Dessa forma é garantido que sempre irá ser printado o tempo a cada minuto
		minutes++;
		cout << "---" << minutes << " MIN @"<< rate << "x ---" << endl;
		//cout << "steps:"<<report->qtd_of_steps<<" time:" << checkTimeMin() << endl;
	}

	report->qtd_of_steps++;

	return situation;
}

int Arbiter::checkTimeMin(){
	return minutes;
}

unsigned int Arbiter::checkTimeMs(){
	return report->qtd_of_steps * SIMULATION_TIME_STEP*1000;//steps * simulation_step_size in ms
}

int Arbiter::getSteps(){
	return report->qtd_of_steps;
}

void Arbiter::position_objects_after_goal_team_1(){
	vector<btVector3> robots;

	robots.push_back(btVector3(55,4,45));
	robots.push_back(btVector3(35,4,30));
	robots.push_back(btVector3(15,4,SIZE_DEPTH- 55));
	robots.push_back(btVector3(SIZE_WIDTH-55,4,85));
	robots.push_back(btVector3(SIZE_WIDTH-25,4,SIZE_DEPTH - SIZE_DEPTH/2.5 + 20));
	robots.push_back(btVector3(SIZE_WIDTH-15,4,55));

	physics->setBallPosition(btVector3( (SIZE_WIDTH/2.0)+10 , 2.0, SIZE_DEPTH/2.0));
	physics->setRobotsPosition(robots);
}

void Arbiter::position_objects_after_goal_team_2(){
	vector<btVector3> robots;

	robots.push_back(btVector3(55,4,45));
	robots.push_back(btVector3(35,4,30));
	robots.push_back(btVector3(15,4,SIZE_DEPTH- 55));
	robots.push_back(btVector3(SIZE_WIDTH-55,4,85));
	robots.push_back(btVector3(SIZE_WIDTH-25,4,SIZE_DEPTH - SIZE_DEPTH/2.5 + 20));
	robots.push_back(btVector3(SIZE_WIDTH-15,4,55));


	physics->setBallPosition(btVector3( (SIZE_WIDTH/2.0)+10 , 2.0, SIZE_DEPTH/2.0));
	physics->setRobotsPosition(robots);
}
