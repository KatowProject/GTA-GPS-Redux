#include "GPS.h"

void GPS::Run()
{
	logger = util::Logger(cfg.LOGFILE_ENABLED);

	plugin::Events::gameProcessEvent += [this]() { this->GameEventHandle(); };

	plugin::Events::drawRadarOverlayEvent += [this]() { this->DrawRadarOverlayHandle(); };

	plugin::Events::drawHudEvent += [this]() { this->DrawHudEventHandle(); };

	plugin::Events::reInitGameEvent += [this]() {
		mTrace = nullptr;
		renderMissionRoute = false;
		renderTargetRoute = false;
		rerouteTimer = 0.0f;
		turnAlertPlayed = false;
		arrivedAlertPlayed = false;
	};
}

void GPS::LoadPathNodesSector(int sectorId)
{
	if (sectorId < 0 || sectorId >= 64)
		return;

	void **pPathNodesArray = reinterpret_cast<void **>(reinterpret_cast<uintptr_t>(&ThePaths) + 0x804);
	if (pPathNodesArray[sectorId] != nullptr)
		return;

	auto LoadPathFindData = reinterpret_cast<void(__thiscall *)(void *, int)>(0x452F40);
	LoadPathFindData(&ThePaths, sectorId);
}

void GPS::EnsureAllPathNodesLoaded()
{
	static bool allLoaded = false;
	if (allLoaded)
		return;

	for (int i = 0; i < 64; ++i)
	{
		LoadPathNodesSector(i);
	}
	allLoaded = true;
}

void GPS::doSinglePathSearch(CVector origin, CVector target, CNodeAddress *resultNodes, short &nodesCount, float &gpsDistance, bool oneSideOnly, bool waterPath)
{
	nodesCount = 0;
	gpsDistance = 0.0f;

	ThePaths.DoPathSearch(
		0, origin, CNodeAddress(), target, resultNodes, &nodesCount, MAX_NODE_POINTS, &gpsDistance,
		999999.0f, NULL, 999999.0f,
		oneSideOnly,
		CNodeAddress(), false,
		waterPath
	);

	// Fallback: retry without lane restriction if it failed
	if (nodesCount <= 1 && oneSideOnly)
	{
		ThePaths.DoPathSearch(
			0, origin, CNodeAddress(), target, resultNodes, &nodesCount, MAX_NODE_POINTS, &gpsDistance,
			999999.0f, NULL, 999999.0f,
			false,
			CNodeAddress(), false,
			waterPath
		);
	}
}

void GPS::calculatePath(const CVector& destPosn, short &nodesCount, CNodeAddress *resultNodes, float &gpsDistance)
{
	if (!player)
		return;

	EnsureAllPathNodesLoaded();

	bool inVehicle = player->m_nPedFlags.bInVehicle && (player->m_pVehicle != nullptr);
	bool inBoat = inVehicle && (player->m_pVehicle->m_nVehicleSubClass == VEHICLE_BOAT);
	bool inBmx = inVehicle && (player->m_pVehicle->m_nVehicleSubClass == VEHICLE_BMX);

	bool oneSideOnly = (inVehicle && !inBoat && !inBmx && cfg.RESPECT_LANE_DIRECTION);
	bool waterPath = (inBoat && cfg.ENABLE_WATER_GPS);

	CVector playerPos = player->GetPosition();
	float dx = destPosn.x - playerPos.x;
	float dy = destPosn.y - playerPos.y;
	float totalDist = sqrtf(dx * dx + dy * dy);

	// For short distances, do a single search (the engine can handle it)
	const float SEGMENT_LENGTH = 450.0f;
	if (totalDist <= SEGMENT_LENGTH * 1.2f)
	{
		doSinglePathSearch(playerPos, destPosn, resultNodes, nodesCount, gpsDistance, oneSideOnly, waterPath);
		return;
	}

	// Multi-segment pathfinding for long distances
	// Break the straight-line distance into segments and pathfind each one
	int numSegments = static_cast<int>(ceilf(totalDist / SEGMENT_LENGTH));
	if (numSegments < 2) numSegments = 2;
	if (numSegments > 20) numSegments = 20; // safety cap

	// Build waypoints along the straight line
	CVector waypoints[21]; // numSegments + 1 points
	for (int i = 0; i <= numSegments; ++i)
	{
		float t = static_cast<float>(i) / static_cast<float>(numSegments);
		waypoints[i].x = playerPos.x + dx * t;
		waypoints[i].y = playerPos.y + dy * t;
		waypoints[i].z = playerPos.z + (destPosn.z - playerPos.z) * t;
	}

	// Temporary buffer for each segment's results
	static CNodeAddress segNodes[MAX_NODE_POINTS];
	nodesCount = 0;
	gpsDistance = 0.0f;

	for (int seg = 0; seg < numSegments; ++seg)
	{
		short segCount = 0;
		float segDist = 0.0f;

		doSinglePathSearch(waypoints[seg], waypoints[seg + 1], segNodes, segCount, segDist, oneSideOnly, waterPath);

		if (segCount <= 1)
		{
			// This segment failed. Try a direct search from current concatenated
			// endpoint to the final destination as a last resort.
			if (seg > 0 && nodesCount > 0)
			{
				// Use the position of the last successfully added node as origin
				CPathNode *lastNode = ThePaths.GetPathNode(resultNodes[nodesCount - 1]);
				if (lastNode)
				{
					CVector lastPos = lastNode->GetNodeCoors();
					doSinglePathSearch(lastPos, destPosn, segNodes, segCount, segDist, oneSideOnly, waterPath);
					if (segCount > 1)
					{
						// Skip the first node (overlap with previous segment)
						int startIdx = 1;
						for (int j = startIdx; j < segCount && nodesCount < MAX_NODE_POINTS; ++j)
						{
							resultNodes[nodesCount++] = segNodes[j];
						}
						gpsDistance += segDist;
					}
				}
			}
			// If first segment fails or last resort fails, try full direct path
			if (nodesCount <= 1)
			{
				doSinglePathSearch(playerPos, destPosn, resultNodes, nodesCount, gpsDistance, oneSideOnly, waterPath);
			}
			break;
		}

		if (seg == 0)
		{
			// Copy all nodes from first segment
			int copyCount = (segCount <= MAX_NODE_POINTS) ? segCount : MAX_NODE_POINTS;
			for (int j = 0; j < copyCount; ++j)
			{
				resultNodes[j] = segNodes[j];
			}
			nodesCount = static_cast<short>(copyCount);
		}
		else
		{
			// Append nodes from subsequent segments, skipping the first node
			// (it overlaps with the last node of the previous segment)
			int startIdx = 1;
			for (int j = startIdx; j < segCount && nodesCount < MAX_NODE_POINTS; ++j)
			{
				resultNodes[nodesCount++] = segNodes[j];
			}
		}
		gpsDistance += segDist;
	}
}

void GPS::requestTargetPath(CVector destPosn)
{
	this->calculatePath(destPosn, targetNodesCount, t_ResultNodes.data(), targetDistance);
}

void GPS::requestMissionPath(CVector destPosn)
{
	this->calculatePath(destPosn, missionNodesCount, m_ResultNodes.data(), missionDistance);
}

// Events

void GPS::DrawRadarOverlayHandle()
{
	if (FrontEndMenuManager.m_bMenuActive)
		return;

	if (!util::NavEnabled(this->cfg, player))
		return;

	if (renderTargetRoute)
	{
		this->renderPath(targetTracePos, -1, false, targetNodesCount, t_ResultNodes.data(), targetDistance,
						 t_LineVerts.data());
	}

	if (renderMissionRoute)
	{
		try
		{
			this->renderMissionTrace(mTrace);
		}
		catch (const std::exception &e)
		{
			logger.Log(e.what());
			renderMissionRoute = false;
		}
	}
}

void GPS::GameEventHandle()
{
	if (FrontEndMenuManager.m_bMenuActive)
		return;

	player = FindPlayerPed(0);
	if (!player)
		return;

	if (!util::NavEnabled(this->cfg, player))
	{
		renderMissionRoute = false;
		renderTargetRoute = false;
		return;
	}

	// Advance animation phase
	if (cfg.ENABLE_ANIM_LINE)
	{
		animPhase += cfg.ANIM_SPEED * CTimer::ms_fTimeStep * 0.02f;
		if (animPhase > 6.2831853f) // 2*PI
			animPhase -= 6.2831853f;
	}

	renderTargetRoute = FrontEndMenuManager.m_nTargetBlipIndex == 0 ? false : true;

	if (!mTrace)
		renderMissionRoute = false;
	else
	{
		renderMissionRoute = mTrace->m_bInUse;
		if (mTrace->m_nBlipDisplay < 2 ||
			DistanceBetweenPoints(player->GetPosition(), mTrace->m_vecPos) <= cfg.DISABLE_PROXIMITY)
		{
			renderMissionRoute = false;
		}

		if (!renderMissionRoute)
			mTrace = nullptr;
	}

	if (FrontEndMenuManager.m_nTargetBlipIndex &&
		CRadar::ms_RadarTrace[LOWORD(FrontEndMenuManager.m_nTargetBlipIndex)].m_nCounter ==
			HIWORD(FrontEndMenuManager.m_nTargetBlipIndex) &&
		CRadar::ms_RadarTrace[LOWORD(FrontEndMenuManager.m_nTargetBlipIndex)].m_nBlipDisplay &&
		DistanceBetweenPoints(player->GetPosition(),
							  CRadar::ms_RadarTrace[LOWORD(FrontEndMenuManager.m_nTargetBlipIndex)].m_vecPos) <=
			cfg.DISABLE_PROXIMITY)
	{
		if (cfg.ENABLE_AUDIO && cfg.AUDIO_ARRIVED && !arrivedAlertPlayed)
		{
			AudioEngine.ReportFrontendAudioEvent(AE_FRONTEND_PART_MISSION_COMPLETE, 0.0f, 1.0f);
			arrivedAlertPlayed = true;
		}
		CRadar::ClearBlip(FrontEndMenuManager.m_nTargetBlipIndex);
		FrontEndMenuManager.m_nTargetBlipIndex = 0;
		renderTargetRoute = false;
	}

	if (FrontEndMenuManager.m_nTargetBlipIndex &&
		CRadar::ms_RadarTrace[LOWORD(FrontEndMenuManager.m_nTargetBlipIndex)].m_nCounter ==
			HIWORD(FrontEndMenuManager.m_nTargetBlipIndex) &&
		CRadar::ms_RadarTrace[LOWORD(FrontEndMenuManager.m_nTargetBlipIndex)].m_nBlipDisplay)
	{
		targetTracePos = CRadar::ms_RadarTrace[LOWORD(FrontEndMenuManager.m_nTargetBlipIndex)].m_vecPos;
		this->requestTargetPath(targetTracePos);
		renderTargetRoute = true;

		// Check off-route detection
		if (cfg.ENABLE_REROUTE && targetNodesCount > 1)
		{
			float offDist = getMinDistanceToRoute(player->GetPosition(), targetNodesCount, t_ResultNodes.data());
			if (offDist > cfg.OFF_ROUTE_DISTANCE)
			{
				rerouteTimer = 2.5f;
				if (cfg.ENABLE_AUDIO && cfg.AUDIO_REROUTE && (CTimer::m_snTimeInMilliseconds - lastRerouteTick > 5000))
				{
					AudioEngine.ReportFrontendAudioEvent(AE_FRONTEND_RADIO_CLICK_ON, 0.0f, 1.0f);
					lastRerouteTick = CTimer::m_snTimeInMilliseconds;
				}
				this->requestTargetPath(targetTracePos);
			}
		}

		// Detect next turn for target route
		if (cfg.ENABLE_NEXT_TURN)
			findNextTurn(targetNodesCount, t_ResultNodes.data());
	}
	else if (!renderMissionRoute)
	{
		arrivedAlertPlayed = false;
	}

	try
	{
		if (CTheScripts::IsPlayerOnAMission())
		{
			float bestDist = std::numeric_limits<float>::infinity();
			tRadarTrace *bestTrace = nullptr;

			for (int i = 0; i < 175; ++i)
			{
				tRadarTrace *trace = &CRadar::ms_RadarTrace[i];
				if (trace && trace->m_nRadarSprite == 0 && trace->m_nBlipDisplay > 1)
				{
					float d = DistanceBetweenPoints(player->GetPosition(), trace->m_vecPos);
					if (d < bestDist)
					{
						bestDist = d;
						bestTrace = trace;
					}
				}
			}

			mTrace = bestTrace;
			renderMissionRoute = bestTrace != nullptr;
			if (renderMissionRoute && mTrace)
			{
				bool valid = true;
				if (mTrace->m_nBlipType == 1)
				{
					if (cfg.ENABLE_MOVING)
					{
						auto *vehicle = CPools::GetVehicle(mTrace->m_nEntityHandle);
						if (vehicle) destVec = vehicle->GetPosition();
						else valid = false;
					}
					else valid = false;
				}
				else if (mTrace->m_nBlipType == 2)
				{
					if (cfg.ENABLE_MOVING)
					{
						auto *ped = CPools::GetPed(mTrace->m_nEntityHandle);
						if (ped) destVec = ped->GetPosition();
						else valid = false;
					}
					else valid = false;
				}
				else if (mTrace->m_nBlipType == 3)
				{
					auto *obj = CPools::GetObject(mTrace->m_nEntityHandle);
					if (obj) destVec = obj->GetPosition();
					else valid = false;
				}
				else if (mTrace->m_nBlipType == 6 || mTrace->m_nBlipType == 8 || mTrace->m_nBlipType == 7 || mTrace->m_nBlipType == 0)
				{
					valid = false;
				}
				else
				{
					destVec = mTrace->m_vecPos;
				}

				if (valid)
				{
					this->requestMissionPath(destVec);

					if (DistanceBetweenPoints(player->GetPosition(), destVec) <= cfg.DISABLE_PROXIMITY)
					{
						if (cfg.ENABLE_AUDIO && cfg.AUDIO_ARRIVED && !arrivedAlertPlayed)
						{
							AudioEngine.ReportFrontendAudioEvent(AE_FRONTEND_PART_MISSION_COMPLETE, 0.0f, 1.0f);
							arrivedAlertPlayed = true;
						}
					}

					// Check off-route detection for mission
					if (cfg.ENABLE_REROUTE && missionNodesCount > 1)
					{
						float offDist = getMinDistanceToRoute(player->GetPosition(), missionNodesCount, m_ResultNodes.data());
						if (offDist > cfg.OFF_ROUTE_DISTANCE)
						{
							rerouteTimer = 2.5f;
							if (cfg.ENABLE_AUDIO && cfg.AUDIO_REROUTE && (CTimer::m_snTimeInMilliseconds - lastRerouteTick > 5000))
							{
								AudioEngine.ReportFrontendAudioEvent(AE_FRONTEND_RADIO_CLICK_ON, 0.0f, 1.0f);
								lastRerouteTick = CTimer::m_snTimeInMilliseconds;
							}
							this->requestMissionPath(destVec);
						}
					}

					// Detect next turn for mission route
					if (cfg.ENABLE_NEXT_TURN)
						findNextTurn(missionNodesCount, m_ResultNodes.data());
				}
				else
				{
					renderMissionRoute = false;
				}
			}
		}
		else
		{
			renderMissionRoute = false;
		}
	}
	catch (const std::exception &e)
	{
		logger.Log(e.what());
		renderMissionRoute = false;
	}

}

void GPS::DrawHudEventHandle()
{
	if (FrontEndMenuManager.m_bMenuActive)
		return;

	if (!cfg.ENABLE_DISTANCE_TEXT)
		return;

	if (!NavEnabled(this->cfg, player))
		return;

	float speed = 16.0f; // default cruising ~60 km/h (16 m/s)
	if (player)
	{
		if (player->m_nPedFlags.bInVehicle && player->m_pVehicle)
		{
			float cur = player->m_pVehicle->m_vecMoveSpeed.Magnitude() * 50.0f;
			if (cur > 3.0f)
				speed = cur;
		}
		else
		{
			float cur = player->m_vecMoveSpeed.Magnitude() * 50.0f;
			if (cur > 1.5f)
				speed = cur;
			else
				speed = 4.0f; // on-foot run speed
		}
	}

	if (renderMissionRoute && mTrace)
	{
		CFont::SetOrientation(ALIGN_CENTER);
		CFont::SetColor(SetupColor(this->mTrace->m_nColour, this->mTrace->m_bFriendly, cfg));
		CFont::SetBackground(false, false);
		CFont::SetWrapx(500.0f);
		CFont::SetScale(0.3f * static_cast<float>(RsGlobal.maximumWidth) / 640.0f,
						0.6f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f);
		CFont::SetFontStyle(FONT_SUBTITLES);
		CFont::SetProportional(true);
		CFont::SetDropShadowPosition(1);
		CFont::SetDropColor(CRGBA(0, 0, 0, 180));

		CVector2D point;
		CRadar::TransformRadarPointToScreenSpace(point, CVector2D(0.0f, -1.0f));

		float dist = DistanceBetweenPoints(FindPlayerCoors(0), destVec);
		std::string str = util::formatDist(dist, cfg.DISTANCE_UNITS, cfg.ENABLE_ETA, speed);

		CFont::PrintString(
			point.x, point.y + 8.0f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f,
			(char *)str.c_str());
	}

	if (renderTargetRoute && FrontEndMenuManager.m_nTargetBlipIndex)
	{
		CFont::SetOrientation(ALIGN_CENTER);
		CFont::SetColor(cfg.GPS_LINE_CLR);

		CFont::SetBackground(false, false);
		CFont::SetWrapx(500.0f);
		CFont::SetScale(0.3f * static_cast<float>(RsGlobal.maximumWidth) / 640.0f,
						0.6f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f);
		CFont::SetFontStyle(FONT_SUBTITLES);
		CFont::SetProportional(true);
		CFont::SetDropShadowPosition(1);
		CFont::SetDropColor(CRGBA(0, 0, 0, 180));

		CVector2D point;
		CRadar::TransformRadarPointToScreenSpace(point, CVector2D(0.0f, 1.0f));

		float dist = DistanceBetweenPoints(
			CVector(player->GetPosition()),
			CVector(CRadar::ms_RadarTrace[LOWORD(FrontEndMenuManager.m_nTargetBlipIndex)].m_vecPos));

		std::string str = util::formatDist(dist, cfg.DISTANCE_UNITS, cfg.ENABLE_ETA, speed);

		CFont::PrintString(
			point.x, point.y - 20.0f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f,
			(char *)str.c_str());
	}

	// Next turn direction and distance display
	if (cfg.ENABLE_NEXT_TURN && hasNextTurn && (renderTargetRoute || renderMissionRoute))
	{
		CFont::SetOrientation(ALIGN_CENTER);
		CFont::SetColor(CRGBA(255, 255, 255, 220));
		CFont::SetBackground(false, false);
		CFont::SetWrapx(500.0f);
		CFont::SetScale(0.28f * static_cast<float>(RsGlobal.maximumWidth) / 640.0f,
						0.56f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f);
		CFont::SetFontStyle(FONT_SUBTITLES);
		CFont::SetProportional(true);
		CFont::SetDropShadowPosition(1);
		CFont::SetDropColor(CRGBA(0, 0, 0, 200));

		CVector2D turnPoint;
		CRadar::TransformRadarPointToScreenSpace(turnPoint, CVector2D(0.0f, 1.0f));

		// Build turn indicator string
		std::string turnStr;
		if (nextTurnDirection < 0)
			turnStr = "< ";
		else
			turnStr = "> ";

		turnStr += util::formatDist(nextTurnDist, cfg.DISTANCE_UNITS, false, 0.0f);

		float yOffset = renderTargetRoute ? -32.0f : -20.0f;
		CFont::PrintString(
			turnPoint.x, turnPoint.y + yOffset * static_cast<float>(RsGlobal.maximumHeight) / 448.0f,
			(char *)turnStr.c_str());
	}

	// Rerouting notification text
	if (rerouteTimer > 0.0f)
	{
		rerouteTimer -= CTimer::ms_fTimeStep * 0.02f;
		if (rerouteTimer < 0.0f)
			rerouteTimer = 0.0f;

		CFont::SetOrientation(ALIGN_CENTER);
		CFont::SetColor(CRGBA(255, 190, 20, 255));
		CFont::SetBackground(false, false);
		CFont::SetWrapx(500.0f);
		CFont::SetScale(0.30f * static_cast<float>(RsGlobal.maximumWidth) / 640.0f,
						0.60f * static_cast<float>(RsGlobal.maximumHeight) / 448.0f);
		CFont::SetFontStyle(FONT_SUBTITLES);
		CFont::SetProportional(true);
		CFont::SetDropShadowPosition(1);
		CFont::SetDropColor(CRGBA(0, 0, 0, 220));

		CVector2D rPoint;
		CRadar::TransformRadarPointToScreenSpace(rPoint, CVector2D(0.0f, 1.0f));
		float yOffset = renderTargetRoute ? -44.0f : -32.0f;
		CFont::PrintString(
			rPoint.x, rPoint.y + yOffset * static_cast<float>(RsGlobal.maximumHeight) / 448.0f,
			(char *)"REROUTING...");
	}
}

// Rendering

void GPS::renderPath(CVector tracePos, short color, bool friendly, short &nodesCount, CNodeAddress *resultNodes,
					 float &gpsDistance, RwIm2DVertex *lineVerts)
{
	if (nodesCount <= 0)
	{
		return;
	}

	for (unsigned short i = 0; i < nodesCount; i++)
	{
		currentNode = ThePaths.GetPathNode(resultNodes[i]);
		if (!currentNode)
		{
			nodesCount = i;
			break;
		}
		nodePosn = currentNode->GetNodeCoors();

		CRadar::TransformRealWorldPointToRadarSpace(tmpPoint, CVector2D(nodePosn.x, nodePosn.y));
		if (!FrontEndMenuManager.m_bDrawRadarOrMap)
		{
			CRadar::TransformRadarPointToScreenSpace(tmpNodePoints[i], tmpPoint);
		}
		else
		{
			CRadar::LimitRadarPoint(tmpPoint);
			CRadar::TransformRadarPointToScreenSpace(tmpNodePoints[i], tmpPoint);
			tmpNodePoints[i].x *= static_cast<float>(RsGlobal.maximumWidth) / 640.0f;
			tmpNodePoints[i].y *= static_cast<float>(RsGlobal.maximumHeight) / 448.0f;
			CRadar::LimitToMap(&tmpNodePoints[i].x, &tmpNodePoints[i].y);
		}
	}

	if (nodesCount <= 1)
	{
		return;
	}

	CRect scissorRect(0, 0, 0, 0);
	if (!FrontEndMenuManager.m_bDrawRadarOrMap &&
		reinterpret_cast<D3DCAPS9 const *>(RwD3D9GetCaps())->RasterCaps & D3DPRASTERCAPS_SCISSORTEST)
	{
		RECT rect;
		CVector2D posn;
		CRadar::TransformRadarPointToScreenSpace(posn, CVector2D(-1.0f, -1.0f));
		rect.left = static_cast<LONG>(posn.x + 2.0f);
		rect.bottom = static_cast<LONG>(posn.y - 2.0f);
		CRadar::TransformRadarPointToScreenSpace(posn, CVector2D(1.0f, 1.0f));
		rect.right = static_cast<LONG>(posn.x - 2.0f);
		rect.top = static_cast<LONG>(posn.y + 2.0f);
		reinterpret_cast<IDirect3DDevice9 *>(GetD3DDevice())->SetRenderState(D3DRS_SCISSORTESTENABLE, TRUE);
		reinterpret_cast<IDirect3DDevice9 *>(GetD3DDevice())->SetScissorRect(&rect);

		scissorRect.Grow(rect.left, rect.right, rect.top, rect.bottom);
	}

	RwRenderStateSet(rwRENDERSTATETEXTURERASTER, NULL);

	CRGBA vColor = SetupColor(color, friendly, cfg);

	vertIndex = 0;
	for (unsigned short i = 0; i < nodesCount - 1; i++)

	{
		dir = tmpNodePoints[i + 1] - tmpNodePoints[i]; // Direction between current node to next node
		angle = atan2(dir.y, dir.x);				   // Convert direction to angle

		float w = cfg.GPS_LINE_WIDTH;
		if (FrontEndMenuManager.m_bDrawRadarOrMap)
		{
			float mp = FrontEndMenuManager.m_fMapZoom - 140.0f;
			if (mp < 140.0f)
				mp = 140.0f;
			else if (mp > 960.0f)
				mp = 960.0f;
			mp = mp / 960.0f + 0.4f;
			w *= mp;
		}

		shift[0].x = cosf(angle - M_PI_2) * w;
		shift[0].y = sinf(angle - M_PI_2) * w;
		shift[1].x = cosf(angle + M_PI_2) * w;
		shift[1].y = sinf(angle + M_PI_2) * w;

		// Only set up vertices for points visible on the radar. If the
		// radar rect is 0 we assume the full screen map is open so we
		// don't apply this optimization.
		if (scissorRect.IsPointInside(tmpNodePoints[i]) ||
			(scissorRect.bottom + scissorRect.top + scissorRect.left + scissorRect.right) == 0)
		{
			// Compute per-segment color (with optional animation)
			CRGBA segColor = vColor;
			if (cfg.ENABLE_ANIM_LINE)
			{
				// Create a wave pattern that flows along the path
				float phase = static_cast<float>(i) * 0.35f - animPhase * 8.0f;
				float wave = (sinf(phase) + 1.0f) * 0.5f; // 0.0 to 1.0
				unsigned char minAlpha = static_cast<unsigned char>(vColor.a * 0.4f);
				segColor.a = static_cast<unsigned char>(minAlpha + (vColor.a - minAlpha) * wave);
			}

			util::Setup2dVertex(lineVerts[vertIndex + 0], tmpNodePoints[i].x + shift[0].x, tmpNodePoints[i].y + shift[0].y, segColor);
			util::Setup2dVertex(lineVerts[vertIndex + 1], tmpNodePoints[i].x + shift[1].x, tmpNodePoints[i].y + shift[1].y, segColor);
			util::Setup2dVertex(lineVerts[vertIndex + 2], tmpNodePoints[i + 1].x + shift[0].x, tmpNodePoints[i + 1].y + shift[0].y, segColor);
			util::Setup2dVertex(lineVerts[vertIndex + 3], tmpNodePoints[i + 1].x + shift[1].x, tmpNodePoints[i + 1].y + shift[1].y, segColor);

			vertIndex += 4;
		}
	}

	if (vertIndex > 0)
	{
		RwIm2DRenderPrimitive(rwPRIMTYPETRISTRIP, lineVerts, vertIndex);
	}

	// Render turn arrows at sharp bends
	if (cfg.ENABLE_TURN_ARROWS)
	{
		renderTurnArrows(nodesCount, vColor);
	}

	if (!FrontEndMenuManager.m_bDrawRadarOrMap &&
		reinterpret_cast<D3DCAPS9 const *>(RwD3D9GetCaps())->RasterCaps & D3DPRASTERCAPS_SCISSORTEST)
	{
		reinterpret_cast<IDirect3DDevice9 *>(GetD3DDevice())->SetRenderState(D3DRS_SCISSORTESTENABLE, FALSE);
	}

	if (nodesCount > 0)
	{
		CPathNode *firstNode = ThePaths.GetPathNode(resultNodes[0]);
		if (firstNode)
			gpsDistance += DistanceBetweenPoints(player->GetPosition(), firstNode->GetNodeCoors());
	}
}

void GPS::renderTurnArrows(short nodesCount, CRGBA color)
{
	if (nodesCount < 3)
		return;

	float thresholdRad = cfg.TURN_ANGLE_THRESHOLD * 3.14159265f / 180.0f;

	// Brighten arrow color for visibility
	CRGBA arrowClr(
		static_cast<unsigned char>(std::min(255, color.r + 80)),
		static_cast<unsigned char>(std::min(255, color.g + 80)),
		static_cast<unsigned char>(std::min(255, color.b + 80)),
		255
	);

	for (unsigned short i = 1; i < nodesCount - 1; i++)
	{
		CVector2D d1 = tmpNodePoints[i] - tmpNodePoints[i - 1];
		CVector2D d2 = tmpNodePoints[i + 1] - tmpNodePoints[i];

		float a1 = atan2f(d1.y, d1.x);
		float a2 = atan2f(d2.y, d2.x);

		float angleDiff = a2 - a1;
		// Normalize to [-PI, PI]
		while (angleDiff > 3.14159265f) angleDiff -= 6.2831853f;
		while (angleDiff < -3.14159265f) angleDiff += 6.2831853f;

		if (fabsf(angleDiff) < thresholdRad)
			continue;

		// Draw a chevron/arrow at the turn point, pointing in the direction of the new segment
		float arrowAngle = a2;
		float sz = cfg.ARROW_SIZE;
		if (FrontEndMenuManager.m_bDrawRadarOrMap)
		{
			float mp = FrontEndMenuManager.m_fMapZoom - 140.0f;
			if (mp < 140.0f) mp = 140.0f;
			else if (mp > 960.0f) mp = 960.0f;
			mp = mp / 960.0f + 0.4f;
			sz *= mp;
		}

		// Triangle vertices: tip + two base points
		RwIm2DVertex arrowVerts[3];

		float tipX = tmpNodePoints[i].x + cosf(arrowAngle) * sz * 2.5f;
		float tipY = tmpNodePoints[i].y + sinf(arrowAngle) * sz * 2.5f;

		float baseL_X = tmpNodePoints[i].x + cosf(arrowAngle + 2.5f) * sz * 1.8f;
		float baseL_Y = tmpNodePoints[i].y + sinf(arrowAngle + 2.5f) * sz * 1.8f;

		float baseR_X = tmpNodePoints[i].x + cosf(arrowAngle - 2.5f) * sz * 1.8f;
		float baseR_Y = tmpNodePoints[i].y + sinf(arrowAngle - 2.5f) * sz * 1.8f;

		util::Setup2dVertex(arrowVerts[0], tipX, tipY, arrowClr);
		util::Setup2dVertex(arrowVerts[1], baseL_X, baseL_Y, arrowClr);
		util::Setup2dVertex(arrowVerts[2], baseR_X, baseR_Y, arrowClr);

		RwIm2DRenderPrimitive(rwPRIMTYPETRILIST, arrowVerts, 3);
	}
}

void GPS::findNextTurn(short nodesCount, CNodeAddress *resultNodes)
{
	hasNextTurn = false;
	if (nodesCount < 3 || !player)
		return;

	float thresholdRad = cfg.TURN_ANGLE_THRESHOLD * 3.14159265f / 180.0f;

	// Find the first sharp turn ahead in the path
	for (int i = 1; i < nodesCount - 1; i++)
	{
		CPathNode *prevNode = ThePaths.GetPathNode(resultNodes[i - 1]);
		CPathNode *currNode = ThePaths.GetPathNode(resultNodes[i]);
		CPathNode *nextNode = ThePaths.GetPathNode(resultNodes[i + 1]);

		if (!prevNode || !currNode || !nextNode)
			continue;

		CVector p0 = prevNode->GetNodeCoors();
		CVector p1 = currNode->GetNodeCoors();
		CVector p2 = nextNode->GetNodeCoors();

		float a1 = atan2f(p1.y - p0.y, p1.x - p0.x);
		float a2 = atan2f(p2.y - p1.y, p2.x - p1.x);

		float angleDiff = a2 - a1;
		while (angleDiff > 3.14159265f) angleDiff -= 6.2831853f;
		while (angleDiff < -3.14159265f) angleDiff += 6.2831853f;

		if (fabsf(angleDiff) >= thresholdRad)
		{
			hasNextTurn = true;
			nextTurnDist = DistanceBetweenPoints(player->GetPosition(), p1);
			nextTurnDirection = (angleDiff < 0) ? 1 : -1; // negative angle = right turn in screen space

			// Audio turn alert when approaching turn (< 65m)
			if (cfg.ENABLE_AUDIO && cfg.AUDIO_TURN_ALERT)
			{
				if (nextTurnDist <= 65.0f && nextTurnDist >= 12.0f)
				{
					if (!turnAlertPlayed)
					{
						AudioEngine.ReportFrontendAudioEvent(AE_FRONTEND_TIMER_COUNT, 0.0f, 1.0f);
						turnAlertPlayed = true;
					}
				}
				else if (nextTurnDist > 80.0f)
				{
					turnAlertPlayed = false;
				}
			}
			return;
		}
	}

	turnAlertPlayed = false;
}

float GPS::getMinDistanceToRoute(CVector playerPos, short nodesCount, CNodeAddress *resultNodes)
{
	if (nodesCount < 2)
		return 0.0f;

	float minDistSq = std::numeric_limits<float>::infinity();

	for (short i = 0; i < nodesCount - 1; ++i)
	{
		CPathNode *n1 = ThePaths.GetPathNode(resultNodes[i]);
		CPathNode *n2 = ThePaths.GetPathNode(resultNodes[i + 1]);
		if (!n1 || !n2)
			continue;

		CVector p1 = n1->GetNodeCoors();
		CVector p2 = n2->GetNodeCoors();

		float vx = p2.x - p1.x;
		float vy = p2.y - p1.y;
		float segLenSq = vx * vx + vy * vy;

		float distSq = 0.0f;
		if (segLenSq < 0.0001f)
		{
			float dx = playerPos.x - p1.x;
			float dy = playerPos.y - p1.y;
			distSq = dx * dx + dy * dy;
		}
		else
		{
			float t = ((playerPos.x - p1.x) * vx + (playerPos.y - p1.y) * vy) / segLenSq;
			if (t < 0.0f) t = 0.0f;
			else if (t > 1.0f) t = 1.0f;

			float projX = p1.x + t * vx;
			float projY = p1.y + t * vy;
			float dx = playerPos.x - projX;
			float dy = playerPos.y - projY;
			distSq = dx * dx + dy * dy;
		}

		if (distSq < minDistSq)
			minDistSq = distSq;
	}

	return sqrtf(minDistSq);
}

void GPS::renderMissionTrace(tRadarTrace *trace)
{
	if (!trace)
		return;

	if (trace->m_nBlipDisplay < 2)
	{
		renderMissionRoute = false;
		return;
	}

	switch (trace->m_nBlipType)
	{
	case 1:
		if (cfg.ENABLE_MOVING)
		{
			auto *vehicle = CPools::GetVehicle(trace->m_nEntityHandle);
			if (!vehicle) { renderMissionRoute = false; return; }
			destVec = vehicle->GetPosition();
		}
		else
		{
			renderMissionRoute = false;
			return;
		}
		break;
	case 2:
		if (cfg.ENABLE_MOVING)
		{
			auto *ped = CPools::GetPed(trace->m_nEntityHandle);
			if (!ped) { renderMissionRoute = false; return; }
			destVec = ped->GetPosition();
		}
		else
		{
			renderMissionRoute = false;
			return;
		}
		break;
	case 3:
	{
		auto *obj = CPools::GetObject(trace->m_nEntityHandle);
		if (!obj) { renderMissionRoute = false; return; }
		destVec = obj->GetPosition();
		break;
	}
	case 6: // Searchlights
	case 8: // Airstripts
	case 0: // NONE???
		return;
	case 7: // Pickups
		renderMissionRoute = false;
		logger.Log("Pickup detected. Not providing GPS navigation!");
		return;
	default:
		destVec = trace->m_vecPos;
		break;
	}

	if (renderMissionRoute)
	{
		this->renderPath(destVec, trace->m_nColour, trace->m_bFriendly, missionNodesCount, m_ResultNodes.data(),
						 missionDistance, m_LineVerts.data());
	}
}
