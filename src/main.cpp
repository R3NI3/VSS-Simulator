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

#include "Header.h"
#include "Simulator.h"
#include "strategies/ModelStrategy.h"
#include "strategies/Strategy.h"
#include "strategies/StrategyBasic.h"
#include "../utils/includes/boost.h"

//Exemplo de estratégia
class StrategyTest : public ModelStrategy{
public:
    void runStrategy(vector<RobotStrategy*> robotStrategiesTeam,vector<RobotStrategy*> robotStrategiesAdv,btVector3 ballPos){
        this->robotStrategiesTeam = robotStrategiesTeam;
        this->robotStrategiesAdv = robotStrategiesAdv;
        this->ballPos = ballPos;

        ModelStrategy::runStrategy();

        for(int i = 0; i < robotStrategiesTeam.size(); i++){
            float leftWheel, rigthWheel;
            leftWheel = 0;
            rigthWheel = 0;
            robotStrategiesTeam[i]->updateCommand(leftWheel,rigthWheel);
        }
    }
};

bool argParse(int argc, char** argv, int *rate, int *qtd_of_goals, bool *develop_mode, int *port, bool *random, int *iaMode);

int main(int argc, char *argv[]){
    int rate = false;
    int qtd_of_goals = 10;
    bool develop_mode = false;
    bool rand_init = false;
    int port;
	 int iaMode;

    if(argParse(argc, argv, &rate, &qtd_of_goals, &develop_mode, &port, &rand_init, &iaMode)){
        Strategy *stratYellowTeam = new Strategy(); //Original strategy
        Strategy *stratBlueTeam = new Strategy(); //Strategy for tests

        Simulator* simulator = new Simulator();
        simulator->runSimulator(argc, argv, stratBlueTeam, stratYellowTeam, rate, qtd_of_goals, develop_mode, port, rand_init, iaMode);
    }else{
        return -1;
    }

	return 0;
}

bool argParse(int argc, char** argv, int *rate, int *qtd_of_goals, bool *develop_mode, int *port, bool *random, int *iaMode){
    namespace bpo = boost::program_options;

    // Declare the supported options.
    bpo::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "(Optional) produce help message")
        ("ia,i", bpo::value<int>()->default_value(0), "IA mode: 0 - disabled, 1 - enable blue")
		  ("rate,r", bpo::value<int>()->default_value(250), "Desired command rate. Default: 250ms")
        ("fixed,f", "(Optional) rate becomes a fixed delay value")
        ("develop,d", "(Optional) turn on the develop mode. the time doesn't count.")
        ("random,a", "(Optional) start objects at random positions, good for training.")
        ("port,p", bpo::value<int>()->default_value(5555), "(Optional) specify port to connect simulator.")
        ("qtd_of_goals,g", bpo::value<std::string>()->default_value("10"), "(Optional) specify the qtd of goals to end the game. 10 to 100");
    bpo::variables_map vm;
    bpo::store(bpo::parse_command_line(argc, argv, desc), vm);
    bpo::notify(vm);

    if (vm.count("help")){
        std::cout << desc << std::endl;
        return false;
    }

	 if (vm.count("ia")){
		  *iaMode = vm["ia"].as<int>();
         std::cout << "IA Mode:" << *iaMode << std::endl;
	 }

    if (vm.count("fixed")){
        *rate = -vm["rate"].as<int>();
         std::cout << "fixed rate" << std::endl;
    } else {
        *rate = vm["rate"].as<int>();
    }

    if (vm.count("develop")){
        *develop_mode = true;
    }

    if (vm.count("random")){
        *random = true;
		  std::cout << "Random positions mode" << std::endl;
    }

    stringstream ss;
    ss << vm["qtd_of_goals"].as<string>();
    ss >> *qtd_of_goals;

    if(*qtd_of_goals < 10){
        *qtd_of_goals = 10;
    }else
    if(*qtd_of_goals > 100){
        *qtd_of_goals = 100;
    }

    *port = vm["port"].as<int>();

    return true;
}
