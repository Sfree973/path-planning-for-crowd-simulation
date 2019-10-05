/*! \file RVOSimulator.h Contains the RVOSimulator class; the main class of the RVO library. */

#ifndef __RVO_SIMULATOR_H__
#define __RVO_SIMULATOR_H__

#include <vector>
#include "vector2.h"

/*! Infinity. A sufficiently large float. */
#define RVO_INFTY 9e9f

// Error messages

/*! The function was successful. */
#define RVO_SUCCESS 0

/*! The simulation is already initialized when an attempt is made to add a goal. */
#define RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_GOAL -10

/*! The simulation is already initialized when an attempt is made to add an agent. */
#define RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_AGENT -11

/*! The simulation is already initialized when an attempt is made to add an obstacle. */
#define RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_OBSTACLE -12

/*! The simulation is already initialized when an attempt is made to add a roadmap vertex. */
#define RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_ROADMAP_VERTEX -13

/*! The simulation is already initialized when an attempt is made to add a roadmap edge. */
#define RVO_SIMULATION_ALREADY_INITIALIZED_WHEN_ADDING_ROADMAP_EDGE -14

/*! The agent defaults have not been set when an attempt is made to add an agent. */
#define RVO_AGENT_DEFAULTS_HAVE_NOT_BEEN_SET_WHEN_ADDING_AGENT -15

/*! The time step has not been set when an attempt is made to do a simulation step. */
#define RVO_TIME_STEP_NOT_SET_WHEN_DOING_STEP -16

/*! The simulation has not been initialized when an attempt is made to do a simulation step. */
#define RVO_SIMULATION_NOT_INITIALIZED_WHEN_DOING_STEP -17

/* \namespace RVO The RVO namespace. */
namespace RVO {
  class KDTree;
  class Agent;
  class Obstacle;
  class RoadmapVertex;
  class Goal;

  /*! The main class of the RVO library. */
  class RVOSimulator {
  public:
    // NOTE: NO PUBLIC CONSTRUCTOR -- singleton model(ע�⣺û�й������캯����������ģʽ)

    /*! Instantiates an RVOSimulator.(ʵ����RVOSimulator)
        \returns The function returns a pointer to the singleton instance.
		�ú�������ָ��һʵ����ָ��*/
    static RVOSimulator* Instance();

    /*! Deconstructor of the RVOSimulator instance.
	    RVOSimulatorʵ��������*/
    ~RVOSimulator();

	

    // Adders
    /*! Adds a goal position to the simulation.���Ŀ��λ��
     \param position The position of the goal	���� λ�ã�Ŀ���λ��
     \returns The function returns the ID of the goal that has been added. It returns an errorcode when the function is called after the simulation has been initialized.
	 \����ֵ �������������Ŀ���ID���������ѱ���ʼ�����᷵�ش������
	  */
    int addGoal(const Vector2& position);

    /*! Sets the default parameters for agents that are subsequently added.
	Ϊ�����ӵ�����������Ĭ�ϲ���
     \param velSampleCountDefault The default number of candidate velocities sampled for the agent in each step of the simulation. The
        running time of the simulation increases linearly with this number.
		��ģ���ÿһ���������ѡ�ٶȵ�Ĭ��ֵ�����������ʱ����������ֳ���������
     \param neighborDistDefault The default distance within which the agent take other agents and obstacles into account in the navigation. The larger this number, the larger the running
        time of the simulation. If the number is too low, the simulation will not be safe.
		Ĭ�Ͼ��룬�������ڴ˾������ڵ���ʱ��������������ϰ��￼�����ڡ��������̫�ͣ�����᲻��ȫ��
     \param maxNeighborsDefault The default maximum number of other agents and obstacles the agent takes into account in the navigation. The larger this number, the larger the running time of
            the simulation. If the number is too low, the simulation will not be safe.
			�ڵ����������彫������������ϰ��￼�����ڵ�Ĭ�ϵ�����������������̫�ͷ���᲻��ȫ
     \param radiusDefault The default radius of the agents
	 �������Ĭ�ϰ뾶
     \param goalRadiusDefault The default goal radius of the agents
	 �������Ĭ��Ŀ��뾶
     \param prefSpeedDefault The default preferred speed of the agents
	 ������Ĭ�ϵ���ѡ�ٶ�
     \param maxSpeedDefault The default maximum speed of the agents
	 �������Ĭ������ٶ�
     \param safetyFactorDefault The default safety factor of the agents. I.e. the weight that is given to the 'time to collision' when penalizing a candidate
        velocity for the agent (vs. the distance to the preferred velocity). The higher this value, the 'safer'
        or the 'shyer' the agent is. The lower this value, the more 'aggressive' and 'reckless' the agent is. Its unit is distance.
		�������Ĭ�ϰ�ȫ���ӡ�����������Ϊ������ͷ�һ����ѡ�ٶȣ����Ǿ������ѡ�ٶȣ�ʱ��������ײʱ�䡣
		���ֵԽ��Խ��ȫ������������������ߡ������ֵԽ�ͣ�������Խ���ö����͡�³ç��
		���ĵ�λ�Ǿ���
     \param maxAccelDefault The default maximum acceleration of the agents
	 �������Ĭ�������ٶ�
     \param velocityDefault The default initial velocity of the agents
	 Ĭ�ϳ�ʼ�ٶ�
     \param orientationDefault The default initial orientation of the agents
	 Ĭ�ϳ�ʼ����
     \param classDefault The default class of the agent   Ĭ����

     */
    void setAgentDefaults( int velSampleCountDefault, float neighborDistDefault, int maxNeighborsDefault, float radiusDefault, float goalRadiusDefault, float prefSpeedDefault, float maxSpeedDefault, float safetyFactorDefault, float maxAccelDefault = RVO_INFTY, const Vector2& velocityDefault = Vector2(0, 0), float orientationDefault = 0, int classDefault = 0 );

    /*! Adds an agent with default parameters to the simulation.
	���������
     \param startPosition ��ʼλ�� The start position of the agent
     \param goalID Ŀ��ID The ID of the goal of the agent
     \returns The function returns the ID of the agent that has been added, and an errorcode if the agent defaults have not been set, or when the function is called after the simulation has been initialized.
     ����������������ID*/
    int addAgent(const Vector2& startPosition, int goalID);

    /*! Adds an agent with specified parameters to the simulation.
	���������ľ������
     \param startPosition The start position of the agent
     \param goalID The ID of the goal of the agent
     \param velSampleCount The number of candidate velocities sampled for the agent in each step of the simulation. The
        running time of the simulation increases linearly with this number.
     \param neighborDist The distance within which the agent take other agents and obstacles into account in the navigation. The larger this number, the larger the running
        time of the simulation. If the number is too low, the simulation will not be safe.
     \param maxNeighbors The maximum number of other agents and obstacles the agent takes into account in the navigation. The larger this number, the larger the running time of
            the simulation. If the number is too low, the simulation will not be safe.
     \param radius The radius of the agent
     \param goalRadius The goal radius of the agent; an agent is said to have reached its goal when it is within a distance goalRadius from its goal position.
     \param prefSpeed The preferred speed of the agent
     \param maxSpeed The maximum speed of the agent
     \param safetyFactor The safety factor of the agent. I.e. the weight that is given to the 'time to collision' when penalizing a candidate
        velocity for the agent (vs. the distance to the preferred velocity). The higher this value, the 'safer'
        or the 'shyer' the agent is. The lower this value, the more 'aggressive' and 'reckless' the agent is. Its unit is distance.
     \param maxAccel The maximum acceleration of the agent
     \param velocity The initial velocity of the agent
     \param orientation The initial orientation of the agent
     \param classID The class of the agent; the class of an agent does not affect the simulation, but can be used in external applications to distinguish among agents
     \returns The function returns the ID of the agent that has been added. It returns an errorcode when the function is called after the simulation has been initialized.
     */
    int addAgent(const Vector2& startPosition, int goalID, int velSampleCount, float neighborDist, int maxNeighbors, float radius, float goalRadius, float prefSpeed, float maxSpeed, float safetyFactor, float maxAccel = RVO_INFTY, const Vector2& velocity = Vector2(0, 0), float orientation = 0, int classID = 0 );

    /*! Adds a line segment obstacle to the simulation.
	Ϊģ����������Էָ���ϰ���
     \param point1 ��һ���˵��λ��The position of the first endpoint of the line segment obstacle
     \param point2 �ڶ����˵��λ��The position of the second endpoint of the line segment obstacle
     \returns The function returns the ID of the obstacle that has been added. It returns an errorcode when the function is called after the simulation has been initialized.
     ����������ϰ����ID*/
    int addObstacle(const Vector2& point1, const Vector2& point2);

    /*! Sets for which radius the mutually visible vertices of the roadmap should automatically be connected by edges when the simulation is initialized. The radius specifies the tolerated distance to obstacles of the line between two roadmap vertices in order for them to be mutually visible. Default is -1.
     \param automaticRadius If non-negative, the mutually visible vertices for the specified radius will be connected in addition to manually specified edges. If negative, no edges will created in addition to the manually specified ones.
    */
    void setRoadmapAutomatic(float automaticRadius);

    /*! Adds a vertex to the roadmap of environment. The roadmap is used for global planning for the agents around obstacles.
	��Ӷ��㵽������·��ͼ��·��ͼ�������������ʱȫ�ֹ滮
     \param position The position of the roadmap vertex·��ͼ�����λ��
     \returns The function returns the ID of the roadmap vertex that has been added. It returns an errorcode when the function is called after the simulation has been initialized.
    �������������ͼ�����ID*/
    int addRoadmapVertex(const Vector2& position);

    /*! Adds an edge between two vertices of the roadmap. The roadmap is undirected.
	���·��ͼ��������֮��ıߡ�·��ͼ�������
     \param vertexID1 �ߵĵ�һ�������IDThe ID of the first vertex of the edge
     \param vertexID2 �ߵĵڶ��������IDThe ID of the second vertex of the edge
     \returns The function returns #RVO_SUCCESS when successful, and an errorcode when the function is called after the simulation has been initialized.
	 �ɹ�ʱ����#RVO_SUCCESS 0*/
    int addRoadmapEdge(int vertexID1, int vertexID2);

    /*! Processes the input (goals, agents, obstacles and roadmap) of the simulation. After initialization, no content can be added anymore to the environment. */
    void initSimulation();

    /*! Causes the simulator to take another simulation step. Modifies position, velocity and orientation of agents. Updates the global time of the simulation, and sets a 'reached goal' flag when all of the agents have reached their goal.
	����ģ����ѡ����һ��ģ�ⲽ���޸�λ�á��ٶȡ����򡣸���ȫ��ʱ�䣬��ȫ������Ŀ��ʱ���á�����Ŀ�ꡱ��־
     \returns The function returns #RVO_SUCCESS on success, and an errorcode when the simulation has not been initialized, or when the global parameters have not been set.*/
    int doStep();

    /*! \returns The function returns true when all agents have reached their goal. Returns false otherwise.
	���أ������������嵽��Ŀ�귵��true����������false*/
    bool getReachedGoal() const;

	//added by myself
	/*--------------------------------------------------------------------*/
	//ĳһ�����嵽��Ŀ�����Ҫ�Ĳ���
	void JudgeReachedGoal(int agentID) ;

	//�ж��Ƿ�ȫ������Ŀ��
	void IsAllAtGoal();
	

	/*-----------------------------------------------------------------*/



    /*! \returns The function returns the current global time of the simulation. Is initially 0.
	����ģ�����ĵ�ǰȫ��ʱ�䡣��ʼ��Ϊ0*/
    float getGlobalTime() const;

    // Global Getter/Setter 's

    /*! \returns The function returns the currently set time step of the simulation.
	���ص�ǰ���õ�ʱ�䲽*/
    float getTimeStep() const;

    /*! Sets the time step of the simulation.����ʱ�䲽
        \param stepSize The new time step of the simulation. ģ�����µ�ʱ�䲽*/
    void setTimeStep( float stepSize );


    // Agent Getter/Setter 's

    /*! \returns The function returns the number of agents in the simulation.
	����������ĸ���*/
    int getNumAgents() const;

    /*! Retrieving whether an agent has reached its goal.�����������Ƿ��Ѿ���������Ŀ��
        \param agentID The agent's ID �������ID
        \returns The function returns true when the specified agent has reached its goal. Returns false otherwise.
		���ض������嵽��Ŀ��ʱ����true�������������false*/
    bool getAgentReachedGoal(int agentID) const;



    /*! Retrieving the current position of an agent.��������ĵ�ǰλ��
        \param agentID The agent's ID
        \returns The function returns the current position of the specified agent. �����ض�������ĵ�ǰλ��*/
    const Vector2& getAgentPosition(int agentID) const;

    /*! Sets the current position of an agent.����������ĵ�ǰλ��
        \param agentID The agent's ID
        \param position The new position of the agent ��λ��*/
    void setAgentPosition(int agentID, const Vector2& position);


    /*! Retrieving the current velocity of an agent.���������嵱ǰ�ٶ�
        \param agentID The agent's ID
        \returns The function returns the current velocity of the specified agent. */
    const Vector2& getAgentVelocity(int agentID) const;

    /*! Sets the current velocity of an agent.���õ�ǰ�ٶ�
        \param agentID The agent's ID
        \param velocity The new velocity of the agent */
    void setAgentVelocity(int agentID, const Vector2& velocity);


    /*! Retrieving the radius of an agent.����������İ뾶
        \param agentID The agent's ID
        \returns The function returns the radius of the specified agent.���ؾ���������İ뾶 */
    float getAgentRadius(int agentID) const;

    /*! Sets the radius of an agent.���ð뾶
        \param agentID The agent's ID
        \param radius The new radius of the agent */
    void setAgentRadius(int agentID, float radius);


    /*! Retrieving the sample number of an agent����һ�������������Ŀ
        \param agentID The agent's ID
        \returns The function returns the currently set number of candidate velocities that is sampled for the specified agent in each step of the simulation.
		�������ص�ǰ��ѡ�ٶȵ����ø���*/
    int getAgentVelSampleCount(int agentID) const;

    /*! Sets the number of candidate velocities that is sampled for the specified agent in each step of the simulation.
        \param agentID The agent's ID
        \param samples The new number of samples.
        */
    void setAgentVelSampleCount(int agentID, int samples);


    /*! Retrieving the neighbor distance of an agent����������ھӾ���
        \param agentID The agent's ID
        \returns The function returns the currently set neighbor distance of the specified agent.
		���ؾ���������ĵ�ǰ�ھӾ���*/
    float getAgentNeighborDist(int agentID) const;

    /*! Sets the neighbor distance of the specified agent.���þ�����������ھӾ���
        \param agentID The agent's ID
        \param distance The new neighbor distance.�µ��ھӾ���
        */
    void setAgentNeighborDist(int agentID, float distance);


    /*! Retrieving the maximum number of neighbors of an agent.�����ھӵ��������
        \param agentID The agent's ID
        \returns The function returns the currently set maximum number of neighbors of the specified agent.
		���ؾ��������嵱ǰ�ھӵ��������*/
    int getAgentMaxNeighbors(int agentID) const;

    /*! Sets the maximum number of neighbors of the specified agent.���þ�����������ھ��������
        \param agentID The agent's ID
        \param maximum The new maximum number of neighbors.
        */
    void setAgentMaxNeighbors(int agentID, int maximum);


    /*! Retrieving the class of an agent.�������������
        \param agentID The agent's ID
        \returns The function returns the class of the specified agent. */
    int getAgentClass(int agentID) const;

    /*! Sets the class of an agent.
        \param agentID The agent's ID
        \param classID The new class of the agent
        */
    void setAgentClass(int agentID, int classID);


    /*! Retrieving the current orientation of an agent.��ǰ������ķ���
        \param agentID The agent's ID
        \returns The function returns the current orientation of the specified agent.*/
    float getAgentOrientation(int agentID) const;

    /*! Sets the current orientation of an agent.���������巽��
        \param agentID The agent's ID
        \param orientation The new orientation of the agent
        */
    void setAgentOrientation(int agentID, float orientation);


    /*! Retrieving the goal of an agent.�������Ŀ��
        \param agentID The agent's ID
        \returns The function returns the ID of the goal of the specified agent.
		����Ŀ��ID*/
    int getAgentGoal(int agentID) const;

    /*! Sets the goal of an agent.����Ŀ��
        \param agentID The agent's ID
        \param goalID The ID of the new goal of the agent
        */
    void setAgentGoal(int agentID, int goalID);

    /*! Retrieving the goal radius of an agent.�����Ŀ��뾶
        \param agentID The agent's ID
        \returns The function returns the goal radius of the specified agent. */
    float getAgentGoalRadius(int agentID) const;

    /*! Sets the goal radius of an agent.����Ŀ��뾶
        \param agentID The agent's ID
        \param goalRadius The new goal radius of the agent
        */
    void setAgentGoalRadius(int agentID, float goalRadius);


    /*! Retrieving the preferred speed of an agent.������ѡ�ٶ�
        \param agentID The agent's ID
        \returns The function returns the preferred speed of the specified agent. */
    float getAgentPrefSpeed(int agentID) const;

    /*! Sets the preferred speed of an agent.������ѡ�ٶ�
        \param agentID The agent's ID
        \param prefSpeed The new preferred speed of the agent
        */
    void setAgentPrefSpeed(int agentID, float prefSpeed);


    /*! Retrieving the maximum speed of an agent.��������ٶ�
        \param agentID The agent's ID
        \returns The function returns the maximum speed of the specified agent. */
    float getAgentMaxSpeed(int agentID) const;

    /*! Sets the maximum speed of an agent.��������ٶ�
        \param agentID The agent's ID
        \param maxSpeed The new maximum speed of the agent
        */
    void setAgentMaxSpeed(int agentID, float maxSpeed);


    /*! Retrieving the maximum acceleration of an agent.���������ٶ�
        \param agentID The agent's ID
        \returns The function returns the acceleration of the specified agent. */
    float getAgentMaxAccel(int agentID) const;

    /*! Sets the maximum acceleration of an agent.���������ٶ�
        \param agentID The agent's ID
        \param maxAccel The new maximum acceleration of the agent
        */
    void setAgentMaxAccel(int agentID, float maxAccel);


    /*! Retrieving the safety factor of an agent.������ȫ����
        \param agentID The agent's ID
        \returns The function returns the safety factor of the specified agent. */
    float getAgentSafetyFactor(int agentID) const;

    /*! Sets the safety factor of an agent.���ð�ȫ����
        \param agentID The agent's ID
        \param safetyFactor The new safety factor of the agent
        */
    void setAgentSafetyFactor(int agentID, float safetyFactor);

    // Goal getters
    /*! \returns The function returns the number of goals in the simulation.
	����Ŀ������*/
    int getNumGoals() const;

    /*! Retrieving the position of a goal.����Ŀ��λ��
        \param goalID The goal's ID
        \returns The function returns the position of the specified goal. */
    const Vector2& getGoalPosition(int goalID) const;

    /*! Retrieving the number of roadmap vertices connected to a goal.��������Ŀ���RoadMap�������
        \param goalID The goal's ID
        \returns The function returns the number of vertices in the roadmap that are neighbors of the specified goal. */
    int getGoalNumNeighbors(int goalID) const;

    /*! Retrieving the ID of a roadmap vertex connected to a goal.��������Ŀ���RoadMap�����ID
        \param goalID The goal's ID
        \param neighborNr The neighbor number
        \returns The ID of the vertex in the roadmap that is the neighborNr'th neighbor of the specified goal. */
    int getGoalNeighbor(int goalID, int neighborNr) const;


    // Obstacle Getter/Setter 's

    /*! \returns The function returns the number of obstacles in the simulation.
	�����ϰ�������*/
    int getNumObstacles() const;

    /*! Retrieving the first endpoint of an obstacle.����һ���ϰ���ĵ�һ���˵�
        \param obstacleID The obstacle's ID
        \returns The function returns the position of the first endpoint of the specified obstacle.
		���ص�һ���˵�λ��*/
    const Vector2& getObstaclePoint1(int obstacleID) const;

    /*! Retrieving the second endpoint of an obstacle.����һ���ϰ���ĵڶ����˵�
        \param obstacleID The obstacle's ID
        \returns The function returns the position of the second endpoint of the specified obstacle.
		���صڶ����˵�λ��*/
    const Vector2& getObstaclePoint2(int obstacleID) const;


    // Roadmap Getters/Setter 's

    /*! \returns The function returns the number of vertices in the roadmap of the simulation.
	����RoadMap�˵�����*/
    int getNumRoadmapVertices() const;

    /*! Retrieving the position of a vertex in the roadmap.�����˵�λ��
        \param vertexID The vertex' ID
        \returns The function returns the position of the specified roadmap vertex. */
    const Vector2& getRoadmapVertexPosition(int vertexID) const;

    /*! Retrieving the number of roadmap vertices connected to a vertex in the roadmap.
	�������ӵ�һ�����������RoadMap���������
        \param vertexID The vertex' ID
        \returns The function returns the number of neighbors of the specified vertex in the roadmap. */
    int getRoadmapVertexNumNeighbors(int vertexID) const;

    /*! Retrieving the ID of a roadmap vertex connected to a vertex in the roadmap.
	�������ӵ�һ�������RoadMap�����ID
        \param vertexID The vertex' ID
        \param neighborNr The neighbor number�ھ�����
        \returns The ID of the vertex in the roadmap that is the neighborNr'th neighbor of the specified roadmap vertex. */
    int getRoadmapVertexNeighbor(int vertexID, int neighborNr) const;

	int _g[1000];
  private:
    /* Private constructor: used to implement Singleton Pattern
	˽�й��캯��������ʵ�ֵ���ģʽ
    */
    RVOSimulator();

    /* Singleton pointer to the RVOSimulator����ָ�룬ָ��RVOSimulation
    */
    static RVOSimulator* _pinstance;

    /* Set of goals in the simulationĿ�꼯 */
    std::vector<RVO::Goal*> _goals;

    /* Set of agents in the simulation �����弯*/
    std::vector<RVO::Agent*> _agents;

    /* Set of obstacles in the simulation�ϰ��Ｏ*/
    std::vector<RVO::Obstacle*>  _obstacles;

    /* The set of roadmap vertices.RoadMap���㼯 */
    std::vector<RVO::RoadmapVertex*> _roadmapVertices;

    /* The datastructure used for efficient neighbor searching ���ڸ�Ч�ھ����������ݽṹ*/
    KDTree* _kdTree;

    /* The default agent parameters for newly added agents 
	����Ӵ����Ĭ�ϴ������*/
    Agent* _defaultAgent;

    /* Radius for which the mutually visible roadmap vertices should automatically be connected by edges. If negative, roadmap vertices will not automatically be connected.
	�໥�ɼ���RoadMap����İ뾶���ڰ뾶֮�ڵ����Զ����ӳ��ߡ�����Ǹ��������㲻���Զ�����*/
    float _automaticRadius;

    /* The current time for the simulatorģ�����ĵ�ǰʱ�� */
    float  _globalTime;

    /* Boolean which reports if all agents are at their goal during this time step.
	�������ͣ������Ƿ���ʱ�䲽�����������嶼�ѵ������ǵ�Ŀ��*/
    bool _allAtGoals;

    /* The size of the time step in the simulation 
	ģ����ʱ�䲽�Ĵ�С*/
    float  _timeStep;

    /* A flag storing whether the agent defaults have been set
	һ����־���洢�Ƿ��������Ĭ��ֵ�Ѿ�������*/
    bool _agentDefaultsHaveBeenSet;
    /* A flag storing whether the simulation has been initialized 
	һ����־���洢�Ƿ�ģ�����Ѿ�����ʼ��*/
    bool _simulationHasBeenInitialized;

    friend class Agent;
    friend class RoadmapVertex;
    friend class Obstacle;
    friend class Goal;
    friend class KDTree;
  };

}  // namespace RVO
#endif

/*! \mainpage RVO Library Documentation
\author Jur van den Berg

The RVO Library provides an easy-to-use implementation of the Reciprocal Velocity Obstacle (RVO)
framework for multi-agent simulation. See http://gamma.cs.unc.edu/RVO/ for papers and demos of the technique.
The library automatically uses parallellism if your machine has multiple processors for computing the motions of the agents.

The library is very easy to use. Please follow the following steps to install and use the library.
- \subpage compiling
- \subpage using
- \subpage params

See the documentation on the RVO::RVOSimulator class for an exhaustive list of public functions of the library.

*/

//-----------------------------------------------------------

/*! \page compiling Compiling the RVO Library
In this section we describe how the RVO library is compiled on various platforms.

\section vs2005 Windows - Visual Studio 2005
Perform the following steps to successfully compile the RVO simulation library.
- Unzip the downloaded file into some directory, here referred to as \$DIR.
- Open the solution \$DIR\\RVOLIB\\RVOLIB.sln into Visual Studio 2005, and select the Release configuration (or alternatively ReleaseST if you do not want to use parallellization).
- Build the solution. This creates a library file rvo.lib (or rvo_st.lib for the single threaded version) in the \$DIR\\lib directory, and copies the header files RVOSimulator.h and vector2.h to the \$DIR\\include directory.
- Use the rvo.lib file, along with the header files RVOSimulator.h and vector2.h in a third party project.

\section UNIX UNIX-based system (e.g. Linux, Cygwin, BSD, and OS X)
The INSTALL file in the root distribution directory has detailed instructions on the standard build system used in for this library; in summary:
- Unpack the tarball
- In the tarball directory, run './configure'; for details on how to configure the build for your system, see './configure --help'
- In the same directory, run 'make'
- To install the headers and library, run 'make install'. You may need elevated permissions, depending on the prefix passed to configure
- The flags '-lrvo' and '-Irvo' should be passed to the linker and compiler, respectively, to build your own software using the RVO library

In the section on \ref using "using the RVO simulation library", there is information on how to use the library functionality in your project.

*/

//-----------------------------------------------------------

/*! \page using Using the RVO Library
In this section we describe how the RVO library can be used in your software to simulate agents.

\section rvostructure Structure
A program performing an RVO simulation has the following global structure.

\code
#include "RVOSimulator.h"

int main() {
  // Create a simulator instance
  RVO::RVOSimulator * sim = RVO::RVOSimulator::Instance();

  // Set up the scenario
  setupScenario( sim );

  // Initialize the simulation
  sim->initSimulation();

  // Perform (and manipulate) the simulation
  do {
    updateVisualization( sim );
    sim->doStep();
  } while ( !sim->getReachedGoal() );

  delete sim;
}
\endcode

In order to use the RVO simulator, the user needs to include RVOSimulator.h.
The first step then is to create an instance of an RVOSimulator.
Then, the process consists of two stages. The first stage is specifying the simulation scenario and its parameters.
In the above example progam, this is done in the method setupScenario, which we will discuss below.
The second stage is the actual performing of the simulation. To finalize the scenario setup and enter
the simulation mode, the simulation has to be initialized by calling RVO::RVOSimulator::initSimulation().
Now, the actual simulation can be performed. In the above example program, simulation steps are taken until
all the agents have reached their goals. Note that it is not allowed to call RVO::RVOSimulator::doStep() before initializing the simulation.
During the simulation, the user may want to retrieve information from the simulation for instance to visualize
the simulation. In the above example program, this is done in the method updateVisualization, which we will discuss below.
It is also possible to manipulate the simulation during the simulation, for instance by changing positions, preferred speeds, goals, etc. of the agents.

\section spec Setting up the Simulation Scenario
A scenario that is to be simulated can be set up as follows. A scenario consists of four types of objects:
goals, agents, obstacles and a roadmap to steer the agents around obstacles.
Each of them can be manually specified. The following example creates a scenario with four agents exchanging positions
around a rectangular obstacle in the middle.

\code
void setupScenario( RVO::RVOSimulator * sim ) {
  // Specify global time step of the simulation
  sim->setTimeStep( 0.25f );

  // Specify default parameters for agents that are subsequently added
  sim->setAgentDefaults( 250, 15.0f, 10, 2.0f, 3.0f, 1.0f, 2.0f, 7.5f, 1.0f );

  // Add agents (and simulataneously their goals), specifying their start position and goal ID
  sim->addAgent( RVO::Vector2(-50.0f, -50.0f), sim->addGoal( RVO::Vector2(50.0f, 50.0f) ) );
  sim->addAgent( RVO::Vector2(50.0f, -50.0f), sim->addGoal( RVO::Vector2(-50.0f, 50.0f) ) );
  sim->addAgent( RVO::Vector2(50.0f, 50.0f), sim->addGoal( RVO::Vector2(-50.0f, -50.0f) ) );
  sim->addAgent( RVO::Vector2(-50.0f, 50.0f), sim->addGoal( RVO::Vector2(50.0f, -50.0f) ) );

  // Add (line segment) obstacles, specifying both endpoints of the line segments
  sim->addObstacle( RVO::Vector2(-7.0f, -20.0f), RVO::Vector2(-7.0f, 20.0f) );
  sim->addObstacle( RVO::Vector2(-7.0f, 20.0f), RVO::Vector2(7.0f, 20.0f) );
  sim->addObstacle( RVO::Vector2(7.0f, 20.0f), RVO::Vector2(7.0f, -20.0f) );
  sim->addObstacle( RVO::Vector2(7.0f, -20.0f), RVO::Vector2(-7.0f, -20.0f) );

  // Add roadmap vertices, specifying their position
  sim->addRoadmapVertex( RVO::Vector2(-10.0f, -23.0f) );
  sim->addRoadmapVertex( RVO::Vector2(-10.0f, 23.0f) );
  sim->addRoadmapVertex( RVO::Vector2(10.0f, 23.0f) );
  sim->addRoadmapVertex( RVO::Vector2(10.0f, -23.0f) );

  // Do not automatically create edges between mutually visible roadmap vertices
  sim->setRoadmapAutomatic( -1 );

  // Manually specify edges between vertices, specifying the ID's of the vertices the edges connect
  sim->addRoadmapEdge( 0, 1 );
  sim->addRoadmapEdge( 1, 2 );
  sim->addRoadmapEdge( 2, 3 );
  sim->addRoadmapEdge( 3, 0 );
}
\endcode

See the documentation on RVO::RVOSimulator for a full overview of the functionality to specify scenarios.

\section ret Retrieving Information from the Simulation
During the simulation, the user can extract information from the simulation for instance for visualization purposes.
In the example program above, this is done in the updateVisualization method. Here we give an example that simply writes
the positions and orientations of each agent in each time step to the standard output.

\code
void updateVisualization( RVO::RVOSimulator * sim ) {
  // Output the current global time
  std::cout << sim->getGlobalTime() << " ";

  // Output the position and orientation for all the agents
  for (int i = 0; i < sim->getNumAgents(); ++i) {
    std::cout << sim->getAgentPosition( i ) << " " << sim->getAgentOrientation( i ) << " ";
  }

  std::cout << std::endl;
}
\endcode

Using similar functions as the ones used in this example, the user can access information about other parameters of the agents, as well as the global parameters, the obstacles and the roadmap.
See the documentation of the class RVO::RVOSimulator for an exhaustive list of public functions for retrieving simulation information.

\section manip Manipulating the Simulation
During the simulation, the user can manipulate the simulation, for instance by changing the global parameters, or changing the parameters of the agents (causing abrupt different behavior).
It is also possible to give the agents a new position, which make them jump through the scene. See the documentation of the class RVO::RVOSimulator for an exhaustive list of public functions for manipulating the simulation.

It is not allowed to add goals, agents, obstacles or roadmap vertices to the simulation, after the simulation has been initialized by calling RVO::RVOSimulator::initSimulation(). Also, it is impossible to change the position of the goals, obstacles or the roadmap vertices.
*/

//-----------------------------------------------------------

/*! \page params Parameter Description
In this section, we give an overview of all parameters of all objects in the simulation.

\section global Global Parameters
\htmlonly
<table border="0" cellpadding="3" width="100%">
    <tr>
        <td valign="top" width="150"><strong>Parameter</strong></td>
        <td valign="top" width="150"><strong>Type (unit)</strong></td>
        <td valign="top"><strong>Meaning</strong></td>
    </tr>
    <tr>
        <td valign="top">timeStep</td>
        <td valign="top">float (time)</td>
        <td valign="top">The time step of the simulation.</td>
    </tr>
</table>
\endhtmlonly

\section goal Goal Parameters
\htmlonly
<table border="0" cellpadding="3" width="100%">
    <tr>
        <td valign="top" width="150"><strong>Parameter</strong></td>
        <td valign="top" width="150"><strong>Type (unit)</strong></td>
        <td valign="top"><strong>Meaning</strong></td>
    </tr>
    <tr>
        <td valign="top" width="150">position</td>
        <td valign="top" width="150">Vector2 (distance, distance)</td>
        <td valign="top">The position of the goal.</td>
    </tr>
</table>
\endhtmlonly

\section agent Agent Parameters
\htmlonly
<table border="0" cellpadding="3" width="100%">
    <tr>
        <td valign="top" width="150"><strong>Parameter</strong></td>
        <td valign="top" width="150"><strong>Type (unit)</strong></td>
        <td valign="top"><strong>Meaning</strong></td>
    </tr>
    <tr>
        <td valign="top" width="150">position</td>
        <td valign="top" width="150">Vector2 (distance, distance)</td>
        <td valign="top">The (current) position of the agent.</td>
    </tr>
    <tr>
        <td valign="top">goalID</td>
        <td valign="top">int</td>
        <td valign="top">The ID of the goal of the agent.</td>
    </tr>
    <tr>
        <td valign="top">velSampleCount</td>
        <td valign="top">int</td>
        <td valign="top">The number of candidate velocities the
        agent samples in each time step of the simulation. The higher the number,
        the more accurate the simulation. The running time of the simulation
        increases linearly with this number.</td>
    </tr>
    <tr>
        <td valign="top">neighborDist</td>
        <td valign="top">float (distance)</td>
        <td valign="top">The distance within which other agents
        and obstacles are taken into account in selecting a
        velocity. The larger this number, the larger the running
        time of the simulation. If the number is too low, the
        simulation will not be safe.</td>
    </tr>
    <tr>
        <td valign="top">maxNeighbors</td>
        <td valign="top">int</td>
        <td valign="top">
            The maximum number of neighboring agents and obstacles that are taken into account
            in selecting a velocity. The larger this number, the larger the running time of
            the simulation. If the number is too low, the simulation will not be safe.</td>
    </tr>
    <tr>
        <td valign="top">radius</td>
        <td valign="top">float (distance)</td>
        <td valign="top">The radius of the agent.</td>
    </tr>
    <tr>
        <td valign="top">goalRadius</td>
        <td valign="top">float (distance)</td>
        <td valign="top">The radius of the goal region around the goal position of the agent. An agent is defined to have reached its goal when it is within a distance goalRadius from its goal position.</td>
    </tr>
    <tr>
        <td valign="top">prefSpeed</td>
        <td valign="top">float (distance/time)</td>
        <td valign="top">The preferred speed of the agent.</td>
    </tr>
    <tr>
        <td valign="top">maxSpeed</td>
        <td valign="top">float (distance/time)</td>
        <td valign="top">The maximum speed of the agent.</td>
    </tr>
    <tr>
        <td valign="top">safetyFactor</td>
        <td valign="top">float (distance)</td>
        <td valign="top">The weight that is given to the &quot;time to collision&quot; when penalizing a candidate
        velocity for the agent (vs. the distance to the preferred velocity). The higher this value, the &quot;safer&quot;
        or the &quot;shyer&quot; the agent is. The lower this value, the more &quot;aggressive&quot; and &quot;reckless&quot; the agent is. Its unit is distance.</td>
    </tr>
    <tr>
        <td valign="top">maxAccel</td>
        <td valign="top">float (distance/time<sup>2</sup>)</td>
        <td valign="top">The maximum acceleration of the agent.</td>
    </tr>
    <tr>
        <td valign="top" width="150">velocity</td>
        <td valign="top" width="150">Vector2 (distance/time, distance/time)</td>
        <td valign="top">The (current) velocity of the agent.</td>
    </tr>
    <tr>
        <td valign="top">orientation</td>
        <td valign="top">float (radians)</td>
        <td valign="top">The (current) orientation of the agent.</td>
   </tr>
    <tr>
       <td valign="top">class</td>
        <td valign="top">int</td>
        <td valign="top">The class of the agent. The class can be used by a visualizer to distinguish among different classes of agents. Its value does not have any effect on the simulation.</td>
    </tr>
</table>
\endhtmlonly

\section obst Obstacle Parameters
\htmlonly
<table border="0" cellpadding="3" width="100%">
    <tr>
        <td valign="top" width="150"><strong>Parameter</strong></td>
        <td valign="top" width="150"><strong>Type (unit)</strong></td>
        <td valign="top"><strong>Meaning</strong></td>
    </tr>
    <tr>
        <td valign="top" width="150">point1</td>
        <td valign="top" width="150">Vector2 (distance, distance)</td>
        <td valign="top">The first endpoint of the line segment obstacle.</td>
    </tr>
    <tr>
        <td valign="top" width="150">point2</td>
        <td valign="top" width="150">Vector2 (distance, distance)</td>
        <td valign="top">The second endpoint of the line segment obstacle.</td>
    </tr>
</table>
\endhtmlonly

\section roadmap Roadmap Parameters
\htmlonly
<table border="0" cellpadding="3" width="100%">
    <tr>
        <td valign="top" width="150"><strong>Parameter</strong></td>
        <td valign="top" width="150"><strong>Type (unit)</strong></td>
        <td valign="top"><strong>Meaning</strong></td>
    </tr>
    <tr>
        <td valign="top">automaticRadius</td>
        <td valign="top">float (distance)</td>
        <td valign="top">The radius for which the mutually visible vertices of the roadmap should automatically be connected by edges when the simulation is initialized. The radius specifies the tolerated distance to obstacles of the line between two roadmap vertices in order for them to be mutually visible. If negative, no edges will created in addition to the manually specified ones.</td>
    </tr>
</table>
\endhtmlonly

\section vertex Roadmap Vertex Parameters
\htmlonly
<table border="0" cellpadding="3" width="100%">
    <tr>
        <td valign="top" width="150"><strong>Parameter</strong></td>
        <td valign="top" width="150"><strong>Type (unit)</strong></td>
        <td valign="top"><strong>Meaning</strong></td>
    </tr>
    <tr>
        <td valign="top" width="150">position</td>
        <td valign="top" width="150">Vector2 (distance, distance)</td>
        <td valign="top">The position of the roadmap vertex.</td>
    </tr>
</table>
\endhtmlonly

\section edge Roadmap Edge Parameters
\htmlonly
<table border="0" cellpadding="3" width="100%">
    <tr>
        <td valign="top" width="150"><strong>Parameter</strong></td>
        <td valign="top" width="150"><strong>Type (unit)</strong></td>
        <td valign="top"><strong>Meaning</strong></td>
    </tr>
    <tr>
        <td valign="top">vertexID1</td>
        <td valign="top">int</td>
        <td valign="top">The ID of the first roadmap vertex of the edge.</td>
    </tr>
    <tr>
        <td valign="top">vertexID2</td>
        <td valign="top">int</td>
        <td valign="top">The ID of the second roadmap vertex of the edge.</td>
    </tr>
</table>
\endhtmlonly

Note that the edges are undirected.
*/

//-----------------------------------------------------------
