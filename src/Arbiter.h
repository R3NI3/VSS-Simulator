#ifndef _ARBITER_H_
#define _ARBITER_H_

#include "iostream"
#include "VSS-Interface/cpp/interface.h"
#include "Sir.h"
#include "Physics.h"

using namespace std;

//! Classe responsável por verificar ocorrencias como: gol, penalti e falta. Também é responsável por monitorar o tempo e reposicionar os objetos em campo.
class Arbiter{
protected:
	Physics *physics;
	Report *report;
	btVector3 history_ball;
	int ball_count;
	int minutes;
    long int sysLastTime;
    long int simLastTime;

    int fault_team_1_counter;
    int fault_team_2_counter;
    int penalty_team_1_counter;
    int penalty_team_2_counter;
public:
	bool refresh;
	Arbiter();

	void allocPhysics(Physics*);
	void allocReport(Report*);
	int checkWorld();
	int getSteps();
	int checkTimeMin();
	unsigned int checkTimeMs();
    long int sysTimeMS();

	void position_objects_after_goal_team_1();
	void position_objects_after_goal_team_2();
	void penalty_team_1();
	void penalty_team_2();

	int is_fault_team_1();
	int is_fault_team_2();
	int is_penalty_team_1();
	int is_penalty_team_2();
};

#endif // _ARBITER_H_
