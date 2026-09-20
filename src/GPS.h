#pragma once

/*
	Plugin-SDK (Grand Theft Auto) source file
	Authors: GTA Community. See more here
	https://github.com/DK22Pac/plugin-sdk
	Do not delete this comment block. Respect others' work!
*/

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>

#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>

#include "CAudioEngine.h"
#include "CFont.h"
#include "CGeneral.h"
#include "CHudColours.h"
#include "CMenuManager.h"
#include "CObject.h"
#include "CPed.h"
#include "CPickup.h"
#include "CPickups.h"
#include "CPools.h"
#include "CRadar.h"
#include "CStreaming.h"
#include "CTheScripts.h"
#include "CTimer.h"
#include "CVehicle.h"
#include "CWorld.h"
#include "Color.h"
#include "RenderWare.h"
#include "common.h"
#include "d3d9.h"
#include "plugin.h"
#include "extensions/ScriptCommands.h"

#include "util/Bools.h"
#include "util/Config.h"
#include "util/Logger.h"
#include "util/Misc.h"
#include "util/Render.h"

/*
	#define MAX_NODE_POINTS 50000
	#define GPS_LINE_WIDTH  4.0f
	#define GPS_LINE_R  180
	#define GPS_LINE_G  24
	#define GPS_LINE_B  24
	#define GPS_LINE_A  255
	#define MAX_TARGET_DISTANCE 10.0f
*/

#define MAX_NODE_POINTS 5000

class GPS
{
  private:
	void Run();
	void GameEventHandle();
	void DrawHudEventHandle();
	void DrawRadarOverlayHandle();
	void renderMissionTrace(tRadarTrace *trace);
	// Self explanatory.
	void calculatePath(const CVector &destPosn, short &nodesCount, CNodeAddress *resultNodes, float &gpsDistance);
	void doSinglePathSearch(CVector origin, CVector target, CNodeAddress *resultNodes, short &nodesCount, float &gpsDistance, bool oneSideOnly, bool waterPath);
	void requestTargetPath(CVector destPosn);
	void requestMissionPath(CVector destPosn);
	void renderPath(CVector tracePos, short color, bool friendly, short &nodesCount, CNodeAddress *resultNodes,
					float &gpsDistance, RwIm2DVertex *lineVerts);
	void renderTurnArrows(short nodesCount, CRGBA color);
	void findNextTurn(short nodesCount, CNodeAddress *resultNodes);
	float getMinDistanceToRoute(CVector playerPos, short nodesCount, CNodeAddress *resultNodes);

	bool renderMissionRoute = false;
	bool renderTargetRoute = false;
	static void LoadPathNodesSector(int sectorId);
	static void EnsureAllPathNodesLoaded();
	float targetDistance = 0.0f;
	short targetNodesCount = 0;
	float angle = 0.0f;
	unsigned int vertIndex = 0;
	// These will be used for mission objectives
	float missionDistance = 0.0f;
	short missionNodesCount = 0;
	util::Config cfg = util::Config("SA.GPS.CONF.ini");
	util::Logger logger = util::Logger(false);
	CPed *player;
	tRadarTrace *mTrace;
	CPathNode *currentNode;
	CVector destVec;
	CVector targetTracePos;
	CVector2D targetScreen;
	CVector2D tmpPoint;
	CVector2D dir;
	CVector nodePosn;
	CVector2D shift[2];

	// Animated line
	float animPhase = 0.0f;

	// Next turn tracking
	float nextTurnDist = 0.0f;
	int nextTurnDirection = 0; // -1=left, 0=none, 1=right
	bool hasNextTurn = false;

	// Off-route & Rerouting
	float rerouteTimer = 0.0f;
	unsigned int lastRerouteTick = 0;

	// Audio cue states
	bool turnAlertPlayed = false;
	bool arrivedAlertPlayed = false;

	std::array<char, 1024> pathNodesToStream{};
	std::array<int, 50000> pathNodes{};
	std::array<CVector2D, MAX_NODE_POINTS> tmpNodePoints{};
	std::array<CNodeAddress, MAX_NODE_POINTS> t_ResultNodes{};
	std::array<RwIm2DVertex, MAX_NODE_POINTS * 4> t_LineVerts{};
	std::array<CNodeAddress, MAX_NODE_POINTS> m_ResultNodes{};
	std::array<RwIm2DVertex, MAX_NODE_POINTS * 4> m_LineVerts{};

  public:
	inline GPS()
	{
		this->Run();
	}
} GPSLineRedux;