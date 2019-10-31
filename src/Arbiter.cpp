#include "Arbiter.h"
#include <sys/time.h>

Arbiter::Arbiter(){
	refresh = false;
	minutes = 0;
	sysLastTime = -1;
	simLastTime = -1;
	penalty_team_1_counter = penalty_team_2_counter = 0;
	fault_team_1_counter = fault_team_2_counter = 0;
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

int Arbiter::is_fault_team_1() {
    int AREA_X_MIN = 160-15;
    int AREA_Z_MIN = 30;
    int AREA_Z_MAX = 100;

    btVector3 ball = physics->getBallPosition();

    if (ball.getX() < AREA_X_MIN || ball.getZ() < AREA_Z_MIN || ball.getZ() > AREA_Z_MAX) {
        //cerr << "ball not in area:" << ball.getX() << "," << ball.getZ() << endl;
        fault_team_1_counter = 0;
        return false;
    }
    //cerr << "ball in area" << endl;

    AREA_X_MIN-=4;
    AREA_Z_MIN-=4;
    AREA_Z_MAX+=4;

    vector<RobotPhysics*> listRobots = physics->getAllRobots();

    int out_area = 0;
    int i=0;
    for (vector<RobotPhysics*>::iterator it = listRobots.begin(); i<3; it++,i++) {
        btVector3 pos = physics->getRobotPosition(*it);
        if (pos.getX() < AREA_X_MIN || pos.getZ() < AREA_Z_MIN || pos.getZ() > AREA_Z_MAX) {
            out_area++;
				//cout << "Robot not in area:" << pos.getX() << "," << pos.getZ() << endl;
		  } //else
  				//cout << "Robot in area:" << pos.getX() << "," << pos.getZ() << endl;

		  if (out_area>1) break;
    }

    if (out_area < 2) //fault
        fault_team_1_counter++;
	 else
        fault_team_1_counter = 0;

    if (fault_team_1_counter>=5) {
        fault_team_1_counter = 0;
        return true;
    }
    return false;
}


int Arbiter::is_fault_team_2() {

    int AREA_X_MAX = 10+15;
    int AREA_Z_MIN = 30;
    int AREA_Z_MAX = 100;

    btVector3 ball = physics->getBallPosition();

    if (ball.getX() > AREA_X_MAX || ball.getZ() < AREA_Z_MIN || ball.getZ() > AREA_Z_MAX) {
        //cerr << "ball not in area:" << ball.getX() << "," << ball.getZ() << endl;
        fault_team_2_counter = 0;
        return false;
    }
    //cerr << "ball in area" << endl;

    AREA_X_MAX+=4;
    AREA_Z_MIN-=4;
    AREA_Z_MAX+=4;

    vector<RobotPhysics*> listRobots = physics->getAllRobots();

    int out_area = 0;
    for (vector<RobotPhysics*>::iterator it = listRobots.begin()+3; it!=listRobots.end(); it++) {
        btVector3 pos = physics->getRobotPosition(*it);
        if (pos.getX() > AREA_X_MAX || pos.getZ() < AREA_Z_MIN || pos.getZ() > AREA_Z_MAX) {
            out_area++;
            //cout << "Robot not in area:" << pos.getX() << "," << pos.getZ() << endl;
        } //else
            //cout << "Robot in area:" << pos.getX() << "," << pos.getZ() << endl;
    }

    if (out_area < 2)//fault
        fault_team_2_counter++;
    else
        fault_team_2_counter = 0;


    if (fault_team_2_counter>=5) {// 
        fault_team_2_counter = 0;
        return true;
    }
    return false;
}


int Arbiter::is_penalty_team_1() {

    int AREA_X_MAX = 10+15;
    int AREA_Z_MIN = 30;
    int AREA_Z_MAX = 100;

    btVector3 ball = physics->getBallPosition();

    if (ball.getX() > AREA_X_MAX || ball.getZ() < AREA_Z_MIN || ball.getZ() > AREA_Z_MAX) {
        //cerr << "ball not in area:" << ball.getX() << "," << ball.getZ() << endl;
        penalty_team_1_counter = 0;
        return false;
    }
    //cerr << "ball in area" << endl;

    AREA_X_MAX+=4;
    AREA_Z_MIN-=4;
    AREA_Z_MAX+=4;

    vector<RobotPhysics*> listRobots = physics->getAllRobots();

    int out_area = 0;
    int i=0;
    for (vector<RobotPhysics*>::iterator it = listRobots.begin(); i<3; it++,i++) {
        btVector3 pos = physics->getRobotPosition(*it);
        if (pos.getX() > AREA_X_MAX || pos.getZ() < AREA_Z_MIN || pos.getZ() > AREA_Z_MAX) {
            out_area++;
            //cout << "Robot not in area:" << pos.getX() << "," << pos.getZ() << endl;
        } //else
            //cout << "Robot in area:" << pos.getX() << "," << pos.getZ() << endl;
    }

    if (out_area < 2)//penalty
        penalty_team_1_counter++;
    else
        penalty_team_1_counter = 0;


    if (penalty_team_1_counter>=20) {// 1/5 seconds
        penalty_team_1_counter = 0;
        return true;
    }
    return false;
}

int Arbiter::is_penalty_team_2() {
    int AREA_X_MIN = 160-15;
    int AREA_Z_MIN = 30;
    int AREA_Z_MAX = 100;

    btVector3 ball = physics->getBallPosition();

    if (ball.getX() < AREA_X_MIN || ball.getZ() < AREA_Z_MIN || ball.getZ() > AREA_Z_MAX) {
        //cerr << "ball not in area:" << ball.getX() << "," << ball.getZ() << endl;
        penalty_team_2_counter = 0;
        return false;
    }
    //cerr << "ball in area" << endl;

    AREA_X_MIN-=4;
    AREA_Z_MIN-=4;
    AREA_Z_MAX+=4;

    vector<RobotPhysics*> listRobots = physics->getAllRobots();

    int out_area = 0;
    for (vector<RobotPhysics*>::iterator it = listRobots.begin()+3; it!=listRobots.end(); it++) {
        btVector3 pos = physics->getRobotPosition(*it);
        if (pos.getX() < AREA_X_MIN || pos.getZ() < AREA_Z_MIN || pos.getZ() > AREA_Z_MAX) {
            out_area++;
            //cout << "Robot not in area:" << pos.getX() << "," << pos.getZ() << endl;
        } //else
            //cout << "Robot in area:" << pos.getX() << "," << pos.getZ() << endl;
    }

    if (out_area < 2)//penalty
        penalty_team_2_counter++;
    else
        penalty_team_2_counter = 0;

    if (penalty_team_2_counter>=20) {// 1/5 seconds
        penalty_team_2_counter = 0;
        return true;
    }
    return false;
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

    if (is_fault_team_1()) {
        cerr << "---Fault RIGHT ! ---" << endl;
        physics->init_penalty_team_2();
    } else
    if (is_fault_team_2()) {
        cerr << "---Fault LEFT ! ---" << endl;
        physics->init_penalty_team_1();
    }
	 else
    if (is_penalty_team_1()) {
        cerr << "---PENALTY LEFT ! ---" << endl;
        physics->init_penalty_team_2();
    } else
    if (is_penalty_team_2()) {
        cerr << "---PENALTY RIGHT ! ---" << endl;
        physics->init_penalty_team_1();
    }

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

    physics->init_positions();
}

void Arbiter::position_objects_after_goal_team_2(){

	physics->init_positions();
}

void Arbiter::penalty_team_1(){

	physics->init_penalty_team_1();
}

void Arbiter::penalty_team_2(){

	physics->init_penalty_team_2();
}
