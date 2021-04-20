#ifndef _OSMAND_ROUTING_CONFIGURATION_H
#define _OSMAND_ROUTING_CONFIGURATION_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "generalRouter.h"
#include <algorithm>
#include <float.h>
#include <iostream>

struct RoutingRule {
    string tagName;
    string t;
    string v;
    string param;
    string value1;
    string value2;
    string type;
};

struct DirectionPoint {
    double distance = DBL_MAX;
    int32_t pointIndex;
    int32_t x31;
    int32_t y31;
    SHARED_PTR<RouteDataObject> connected;
    std::vector<uint32_t> types;
    std::vector<std::pair<std::string, std::string>> tags;
};

struct RoutingConfiguration {

    const static int DEFAULT_MEMORY_LIMIT = 100;
    const static int DEVIATION_RADIUS = 3000;
    MAP_STR_STR attributes;
    quad_tree<DirectionPoint> directionPoints;
    int directionPointsRadius = 100; // 30 m

    SHARED_PTR<GeneralRouter> router;

	long memoryLimitation;
	float initialDirection;

	int zoomToLoad;
	float heurCoefficient;
	int planRoadDirection;
	string routerName;
	
    // 1.5 Recalculate distance help
    float recalculateDistance;
    time_t routeCalculationTime = 0;

    RoutingConfiguration(float initDirection = -360, int memLimit = DEFAULT_MEMORY_LIMIT) : router(new GeneralRouter()), memoryLimitation(memLimit), initialDirection(initDirection), zoomToLoad(16), heurCoefficient(1), planRoadDirection(0), routerName(""), recalculateDistance(20000.0f) {
    }

    string getAttribute(SHARED_PTR<GeneralRouter> router, string propertyName) {
        if (router->containsAttribute(propertyName)) {
            return router->getAttribute(propertyName);
        }
        return attributes[propertyName];
    }

    void initParams() {
		planRoadDirection = (int) parseFloat(getAttribute(router, "planRoadDirection"), 0);
		heurCoefficient = parseFloat(getAttribute(router, "heuristicCoefficient"), 1);
        recalculateDistance = parseFloat(getAttribute(router, "recalculateDistanceHelp"), 20000);
		// don't use file limitations?
		memoryLimitation = (int)parseFloat(getAttribute(router, "nativeMemoryLimitInMB"), memoryLimitation);
		zoomToLoad = (int)parseFloat(getAttribute(router, "zoomToLoadTiles"), 16);
		//routerName = parseString(getAttribute(router, "name"), "default");
	}
    quad_tree<DirectionPoint> getDirectionPoints() {
        return directionPoints;
    }
};

class RoutingConfigurationBuilder {
private:
    MAP_STR_STR attributes;
    UNORDERED(map)<int64_t, int_pair> impassableRoadLocations;
    std::vector<DirectionPoint> directionPointsBuilder;

public:
    UNORDERED(map)<string, SHARED_PTR<GeneralRouter> > routers;
    string defaultRouter;

    RoutingConfigurationBuilder() : defaultRouter("") {
    }
    
    SHARED_PTR<RoutingConfiguration> build(string router, int memoryLimitMB, const MAP_STR_STR& params = MAP_STR_STR()) {
        return build(router, -360, memoryLimitMB, params);
    }
    
    SHARED_PTR<RoutingConfiguration> build(string router, float direction, long memoryLimitMB, const MAP_STR_STR& params = MAP_STR_STR()) {
        if (routers.find(router) == routers.end()) {
            router = defaultRouter;
        }
        SHARED_PTR<RoutingConfiguration> i = std::make_shared<RoutingConfiguration>();
        if (routers.find(router) != routers.end()) {
            i->router = routers[router]->build(params);
            i->routerName = router;
        }
        attributes["routerName"] = router;
        i->attributes.insert(attributes.begin(), attributes.end());
        i->initialDirection = direction;
        i->memoryLimitation = memoryLimitMB;
        i->initParams();
        
        auto it = impassableRoadLocations.begin();
        for(;it != impassableRoadLocations.end(); it++) {
            i->router->impassableRoadIds.insert(it->first);
        }
        directionPointsBuilder = getTestKyivPoints();
        if (directionPointsBuilder.size() > 0) {
            //std::vector<int32_t> minMaxXY = getMinMax();
            //SkRect rect = SkRect::MakeLTRB(minMaxXY[0], minMaxXY[1], minMaxXY[2], minMaxXY[3]);
            SkRect rect = SkRect::MakeLTRB(0, 0, INT_MAX, INT_MAX);
            i->directionPoints = quad_tree<DirectionPoint>(rect, 14, 0.5);
            for (int j = 0; j < directionPointsBuilder.size(); j++) {
                DirectionPoint & dp = directionPointsBuilder[j];
                //cout << "Point:" << dp.x31 << "," << dp.y31 << std::endl;
                SkRect rectDp = SkRect::MakeLTRB(dp.x31, dp.y31, dp.x31, dp.y31);
                i->directionPoints.insert(dp, rectDp);
            }
        }
        return i;
    }
    
    std::vector<int32_t> getMinMax() {
        int32_t maxX = 0;
        int32_t maxY = 0;
        int32_t minX = INT_MAX;
        int32_t minY = INT_MAX;
        for (DirectionPoint dp : directionPointsBuilder) {
            maxX = dp.x31 > maxX ? dp.x31 : maxX;
            maxY = dp.y31 > maxY ? dp.y31 : maxY;
            minX = dp.x31 < minX ? dp.x31 : minX;
            minY = dp.y31 < minY ? dp.y31 : minY;
        }
        std::vector<int32_t> result{minX, minY, maxX, maxY};
        return result;
    }
   
    std::vector<DirectionPoint> getTestKyivPoints() {
        int i = 0;
        double diffLat = 50.4819 - 50.4004;
        double diffLon = 30.5708 - 30.4196;
        std::vector<std::pair<std::string, std::string>> tags;
        tags.push_back(std::make_pair("motorcar", "no"));
        std::vector<DirectionPoint> result;
        while (i < 2000) {
            DirectionPoint dp;
            double rand1 = ((double) rand() / (RAND_MAX)) + 1;
            double rand2 = ((double) rand() / (RAND_MAX)) + 1;
            double lat = rand1 * diffLat + 50.4004;
            double lon = rand2 * diffLon + 30.4196;
            dp.x31 = get31TileNumberX(lon);
            dp.y31 = get31TileNumberY(lat);
            dp.tags = tags;
            result.push_back(dp);
            i++;
        }
        return result;
    }
    
    UNORDERED(map)<int64_t, int_pair>& getImpassableRoadLocations() {
        return impassableRoadLocations;
    }
    
    bool addImpassableRoad(int64_t routeId, int x31, int y31) {
        if (impassableRoadLocations.find(routeId) == impassableRoadLocations.end()) {
            impassableRoadLocations[routeId] = int_pair(x31, y31);
            return true;
        }
        return false;
    }
    
    SHARED_PTR<GeneralRouter> getRouter(string applicationMode) {
        return routers[applicationMode];
    }
    
    void addRouter(string name, SHARED_PTR<GeneralRouter> router) {
        routers[name] = router;
    }

    void addAttribute(string name, string value) {
        attributes[name] = value;
    }

    void removeImpassableRoad(int64_t routeId) {
        impassableRoadLocations.erase(routeId);
    }
    
    void setDirectionPoints(std::vector<DirectionPoint> directionPoints) {
        directionPointsBuilder = directionPoints;
    }
};

SHARED_PTR<RoutingConfigurationBuilder> parseRoutingConfigurationFromXml(const char* filename);

#endif /*_OSMAND_ROUTING_CONFIGURATION_H*/
