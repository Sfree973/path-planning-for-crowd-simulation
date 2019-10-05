#ifndef _WIN32_WINNT    // Allow use of features specific to Windows XP or later.
#define _WIN32_WINNT 0x0501  // Change this to the appropriate value to target other versions of Windows.
#endif
#define AgentNumber 200
#include <stdio.h>
#include <stdlib.h>
#include<iostream>
using namespace std;

#ifdef _MSC_VER
#include <tchar.h>
#else
#define _TCHAR char
#define _tmain main
#endif

#include "RVOSimulator.h"

//����ϰ���
void AddObstacle(RVO::RVOSimulator * sim, char *filename){
	int i, j;
	FILE *fp1;
	fp1 = fopen(filename, "r");
	float pos_x, pos_y;
	int ob_vertex_num = 0;//�ϰ��������
	int ob_num = 0;//�ϰ�����
	int node_num;//�ڵ����
	char szBuffer[256];
	//std::string str;
	std::vector<int> node_index;
	if (fp1){
		fscanf_s(fp1, "%d", &ob_vertex_num);//���ļ����ϰ��ﶥ�����
		RVO::Vector2 *obstacle_vertex;
		obstacle_vertex = new RVO::Vector2[ob_vertex_num];//������ʸ����ʾ

		for (i = 0; i < ob_vertex_num; i++){
			fscanf_s(fp1, "%f %f", &pos_x, &pos_y);
			obstacle_vertex[i] = RVO::Vector2(pos_x, pos_y);//�ϰ��ﶥ�������
			//sprintf_s(szBuffer, "%f %f\n", obstacle_vertex[i].x(), obstacle_vertex[i].y());
			//str += szBuffer; 

		}
		fscanf_s(fp1, "%d", &ob_num);//���ļ����ϰ������
		for (i = 0; i < ob_num; i++){
			fscanf_s(fp1, "%d", &node_num);//�ϰ����м�������
			node_index.clear();
			int node_index_temp;
			for (j = 0; j < node_num; j++){
				fscanf_s(fp1, "%d", &node_index_temp);
				node_index.push_back(node_index_temp);
			}
			for (j = 0; j < node_num; j++){
				sim->addObstacle(obstacle_vertex[node_index[j]], obstacle_vertex[node_index[(j + 1) % node_num]]);
			}

		}
	}

	fclose(fp1);
	//if (obstacle_vertex)
	//delete[]obstacle_vertex;



}

//���roadmapͼ
void AddRoadMap(RVO::RVOSimulator * sim, char *filename){
	int i;
	float pos_x, pos_y;

	FILE *fp2;
	//add roadmap
	fp2 = fopen(filename, "r");
	int roadmap_vertex_num = 0;
	int roadmap_edge_num = 0;
	if (fp2){
		fscanf_s(fp2, "%d", &roadmap_vertex_num);
		RVO::Vector2 *roadmap_vertex;
		roadmap_vertex = new RVO::Vector2[roadmap_vertex_num];

		for (i = 0; i < roadmap_vertex_num; i++){
			fscanf_s(fp2, "%f %f", &pos_x, &pos_y);
			roadmap_vertex[i] = RVO::Vector2(pos_x, pos_y);
			sim->addRoadmapVertex(roadmap_vertex[i]);
		}
		sim->setRoadmapAutomatic(-1);

		int end1, end2;
		fscanf_s(fp2, "%d", &roadmap_edge_num);
		for (i = 0; i < roadmap_edge_num; i++){
			fscanf_s(fp2, "%d %d", &end1, &end2);
			sim->addRoadmapEdge(end1, end2);
		}
		fclose(fp2);
		if (roadmap_vertex)
			delete[]roadmap_vertex;
	}
}

//�������
void AddAgent_office(RVO::RVOSimulator * sim, char *filename){
	//sim->setAgentDefaults(250, 15.0f, 10, 9.0f, 3.0f, 1.06f, 2.0f, 7.5f, 1.0f);
	//sim->setAgentDefaults(250, 15.0f, 10, 9.0f, 10.0f, 7.0f, 8.0f, 7.5f, 4.0f);
	//sim->setAgentDefaults(250, 15.0f, 10, 9.0f, 40.0f, 1.06f, 2.0f, 7.5f, 1.0f);
	sim->setAgentDefaults(250, 10.0f, 10, 2.0f, 5.0f, 1.500f, 11.5f, 2.5f, 0.5f);//Ч�����

	//sim->setAgentDefaults(160, 12.0f, 7, 4.0f, 18.0f, 2.30f, 2.0f, 7.5f, 1.0f);//Ч�����
	//sim->setAgentDefaults(360, 15.0f, 10, 8.0f, 30.0f, 10.06f, 11.0f, 7.5f, 5.0f);//Ч�����
	//sim->setAgentDefaults(250, 1.50f, 10, 0.20f, 0.30f, 1.0f, 2.0f, 7.5f, 1.0f);
	//sim->setAgentDefaults(250, 8.50f, 10, 2.0f, 3.0f, 5.1f, 2.0f, 7.5f, 1.0f);
	/*  int velSampleCountDefault	��ģ���ÿһ���������ѡ�ٶȵ�Ĭ��ֵ�������ʱ�����������ֳ���������
	float neighborDistDefault	Ĭ�Ͼ��룬�������ڴ˾������ڵ���ʱ��������������ϰ��￼�����ڡ��������̫�ͣ�����᲻��ȫ��
	int maxNeighborsDefault		�ڵ����������彫������������ϰ��￼�����ڵ�Ĭ�ϵ�����������������̫�ͷ���᲻��ȫ��
	float radiusDefault			�������Ĭ�ϰ뾶
	float goalRadiusDefault		�������Ĭ��Ŀ��뾶
	float prefSpeedDefault		������Ĭ�ϵ���ѡ�ٶ�
	float maxSpeedDefault		�������Ĭ������ٶ�
	float safetyFactorDefault	�������Ĭ�ϰ�ȫ���ӡ���������Ϊ������ͷ�һ����ѡ�ٶ�ʱ��������ײʱ��
	float maxAccelDefault		�������Ĭ�������ٶ�
	*/
	FILE *fp3;
	fp3 = fopen(filename, "r");
	if (!fp3) return;

	RVO::Vector2 goal[2];
	//goal[0] = RVO::Vector2(4.530, -24.404f);
	goal[0] = RVO::Vector2(105.000, 0.000f);
	goal[1] = RVO::Vector2(105.000, 300.000f);
	//goal[2] = RVO::Vector2(0.000, -200.000f);
	//goal[3] = RVO::Vector2(10.000, -200.000f);
	//goal[4] = RVO::Vector2(-10.000, -200.000f);
	//goal[5] = RVO::Vector2(10.000, 200.000f);
	//goal[6] = RVO::Vector2(-10.000, 200.000f);
	for (int i = 0; i < 2; i++){
		sim->addGoal(goal[i]);
	}
	float min_dis;
	int goal_index;
	float pos_x, pos_y;
	int i, j;
	float  dis;
	for (i = 0; i < 10; i++){			//AgentNumber����Ϊ100
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		/*min_dis = 1000;
		for (j = 0; j <1; j++){
		dis = (pos_x - goal[j].x())*(pos_x - goal[j].x()) + (pos_y - goal[j].y())*(pos_y - goal[j].y());
		if (sqrt(dis) < min_dis){
		min_dis = sqrt(dis);
		goal_index = j;
		}
		}*/

		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);	//��������壺��ʼλ�ã�Ŀ��ID
		//sim->addAgent(RVO::Vector2(pos_x, pos_y), 0, 250, 10.0f, 10, 1.0f, 1.0f, newperspeed, 2.0f, 7.5f, 1.0f);
	}
	for (int i = 10; i <20; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	}  //˫���ڲ�ע�� ������ע��
	for (int i = 20; i <30; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	}  //˫���ڲ�ע�� ������ע��
	for (int i = 30; i <60; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 1/*goal_index*/);
	}  //˫���ڲ�ע�� ������ע��
	for (int i = 60; i <90; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	}  //˫���ڲ�ע�� ������ע��
	for (int i = 90; i <130; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 1/*goal_index*/);
	}  //˫���ڲ�ע�� ������ע��
	for (int i = 130; i <150; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 1/*goal_index*/);
	}  //˫���ڲ�ע�� ������ע��
	for (int i = 150; i <170; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	}  //˫���ڲ�ע�� ������ע��
	for (int i = 170; i < 180; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	}

	for (int i = 180; i < 190; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	}
	for (int i = 190; i < 200; i++){
		fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
		sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	}
	//for (int i = 40; i < 50; i++){
	//fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
	//sim->addAgent(RVO::Vector2(pos_x, pos_y), 2/*goal_index*/);
	//}  //
	//for (int i = 50; i < 60; i++){
	//fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
	//sim->addAgent(RVO::Vector2(pos_x, pos_y), 0/*goal_index*/);
	//}  //
	//for (int i = 60; i < 70; i++){
	//fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
	//sim->addAgent(RVO::Vector2(pos_x, pos_y), 5/*goal_index*/);
	//}  //
	//for (int i = 70; i < 90; i++){
	//fscanf_s(fp3, "%f %f", &pos_x, &pos_y);
	//sim->addAgent(RVO::Vector2(pos_x, pos_y), 6/*goal_index*/);
	//}  //
	fclose(fp3);
}


void updateVisualization(RVO::RVOSimulator * sim) {
	// Output the current global time
	//std::cout << sim->getGlobalTime() << " ";

	std::string str = "";
	std::string str_pos = "";
	char szBuffer[256];
	float scale_factor = 1.0;
	sprintf(szBuffer, "tms\n");
	str = szBuffer;
	float g[600];
	for (int i = 0; i < sim->getNumAgents(); i++)
	{
		sprintf_s(szBuffer, "c	%d	%d	%f	%f	%f\n", i, 1,
			sim->getAgentPosition(i).x()*scale_factor,
			0.0f,
			sim->getAgentPosition(i).y()*scale_factor
			);
		str += szBuffer;

		sprintf_s(szBuffer, "%f	%f \n",
			sim->getAgentPosition(i).x()*scale_factor,
			sim->getAgentPosition(i).y()*scale_factor
			);
		str_pos += szBuffer;
	}
	str += "\n";
	FILE *fp;
	fp = fopen("../output/3d/��Ⱦ/cmx21.txt", "a");
	if (fp)
	{
		fprintf(fp, str.c_str());
		fclose(fp);
	}

	static int frame = 0;
	// Output the position and orientation for all the agents
	for (int i = 0; i < sim->getNumAgents(); ++i) {
		//std::cout << sim->getAgentPosition( i ) << " " << sim->getAgentOrientation( i ) << " ";
		sprintf_s(szBuffer, "%f %f %f\n", sim->getAgentPosition(i).x(), sim->getAgentPosition(i).y(), sim->getAgentOrientation(i));
		str += szBuffer;
	}
	sprintf_s(szBuffer, "../output/3d/s21/cmx%05d.txt", frame);
	FILE *fp1;
	fp1 = fopen(szBuffer, "w");
	if (fp1){
		fprintf_s(fp1, str_pos.c_str());
		fclose(fp1);
	}
	frame++;
	// std::cout << std::endl;
}


//дͷ�ļ�
void write_file_head(RVO::RVOSimulator * sim)

{
	char szBuffer[256];
	std::string str = "";
	str += "#CrowdAnimationPath\n\n";
	str += "env\n";
	str += "intv	100\n\n";//????

	sprintf_s(szBuffer, "agt %d\n", sim->getNumAgents());
	str += szBuffer;
	int i;
	for (i = 0; i < sim->getNumAgents(); i++)
	{
		sprintf_s(szBuffer, "a	%d\n", i);
		str += szBuffer;
	}
	sprintf_s(szBuffer, "\ngrp 1\n");
	str += szBuffer;
	sprintf_s(szBuffer, "g	1\n\n");
	str += szBuffer;
	FILE *fp;
	fp = fopen("../output/3d/��Ⱦ/cmx21.txt", "w");
	if (fp)
	{
		fprintf(fp, str.c_str());
		fclose(fp);
	}
}

/*void updateVisualization_with_orientation(RVO::RVOSimulator * sim){
std::string str = "";
static int frame = 0;
char szBuffer[256];

for (int i = 0; i < sim->getNumAgents(); ++i){
RVO::Vector2 tempvec = sim->getAgentOrient(i);
sprintf_s(szBuffer, "%f %f %f %f\n", sim->getAgentPosition(i).x(), sim->getAgentPosition(i).y(),
tempvec.x(), tempvec.y());
str += szBuffer;
}
sprintf_s(szBuffer, "../3d_data_output_orientation/sim%05d.txt", frame);
FILE *fp5;
fp5 = fopen(szBuffer, "w");
if (fp5){
fprintf_s(fp5, str.c_str());
fclose(fp5);
}
frame++;

}*/

void setupScenario_test_2(RVO::RVOSimulator * sim){
	//sim->setTimeStep(1.0f);
	sim->setTimeStep(0.25f);

	AddAgent_office(sim, "../input/3d/office200.txt");		//�������
	AddObstacle(sim, "../input/3d/office.txt");			//����ϰ���
	AddRoadMap(sim, "../input/3d/officeroadmap.txt");				//���Roadmapͼ
}



int _tmain(int argc, _TCHAR* argv[])
{
	// Create a simulator instance
	RVO::RVOSimulator * sim = RVO::RVOSimulator::Instance();

	// Set up the scenario
	// setupScenario( sim );
	setupScenario_test_2(sim);
	write_file_head(sim);
	// Initialize the simulation
	sim->initSimulation();

	// Perform (and manipulate) the simulation
	int temp = 0;
	do {
		if (temp % 10 == 0){
			updateVisualization(sim);
			temp++;
		}
		else
			temp++;
		sim->doStep();

	} while (!sim->getReachedGoal());
	updateVisualization(sim);
	cout << sim->getGlobalTime() << endl;
	delete sim;

	printf_s("ok");
	getchar();
	return 0;
}


