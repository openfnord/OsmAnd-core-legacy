#ifndef _OSMAND_ROUTE_SEGMENT_H
#define _OSMAND_ROUTE_SEGMENT_H
#include "CommonCollections.h"
#include "commonOsmAndCore.h"
#include "routeSegmentResult.h"
#include "Logging.h"

struct RouteSegment;

struct RouteSegmentStructure {
public:
	RouteSegmentStructure() {}
	~RouteSegmentStructure() {}
	
	void setNext(const RouteSegment *key, const SHARED_PTR<RouteSegment> value) {
		nextMapping[key] = value;
	}
	
	void setNextLoaded(const RouteSegment *key, const SHARED_PTR<RouteSegment> value) {
		nextLoadedMapping[key] = value;
	}
	
	void setOppositeDirection(const RouteSegment *key, const SHARED_PTR<RouteSegment> value) {
		oppositeDirectionMapping[key] = value;
	}
	
	void setReverseSearch(const RouteSegment *key, const SHARED_PTR<RouteSegment> value) {
		reverseSearchMapping[key] = value;
	}
	
	void setParentRoute(const RouteSegment *key, const SHARED_PTR<RouteSegment> value) {
		parentRouteMapping[key] = value;
	}
	
	void setOpposite(const RouteSegment *key, const SHARED_PTR<RouteSegment> value) {
		oppositeMapping[key] = value;
	}
	
	SHARED_PTR<RouteSegment> getNext(const RouteSegment *key) {
		return nextMapping[key];
	}
	
	SHARED_PTR<RouteSegment> getNextLoaded(const RouteSegment *key) {
		return nextLoadedMapping[key];
	}
	
	SHARED_PTR<RouteSegment> getOppositeDirection(const RouteSegment *key) {
		return oppositeDirectionMapping[key];
	}
	
	SHARED_PTR<RouteSegment> getReverseSearch(const RouteSegment *key) {
		return reverseSearchMapping[key];
	}
	
	SHARED_PTR<RouteSegment> getParentRoute(const RouteSegment *key) {
		return parentRouteMapping[key];
	}
	
	SHARED_PTR<RouteSegment> getOpposite(const RouteSegment *key) {
		return oppositeMapping[key];
	}
	
	void clearData(const RouteSegment *key) {
		nextMapping.erase(key);
		nextLoadedMapping.erase(key);
		oppositeDirectionMapping.erase(key);
		reverseSearchMapping.erase(key);
		parentRouteMapping.erase(key);
		oppositeMapping.erase(key);
	}
	
private:
	UNORDERED_map<const RouteSegment *, SHARED_PTR<RouteSegment>> nextMapping;
	UNORDERED_map<const RouteSegment *, SHARED_PTR<RouteSegment>> nextLoadedMapping;
	UNORDERED_map<const RouteSegment *, SHARED_PTR<RouteSegment>> oppositeDirectionMapping;
	UNORDERED_map<const RouteSegment *, SHARED_PTR<RouteSegment>> reverseSearchMapping;
	UNORDERED_map<const RouteSegment *, SHARED_PTR<RouteSegment>> parentRouteMapping;
	UNORDERED_map<const RouteSegment *, SHARED_PTR<RouteSegment>> oppositeMapping;
};

struct RouteSegment {
    
    // # Represents parent segment for Start & End segment
    static inline SHARED_PTR<RouteSegment> breakSegment = std::make_shared<RouteSegment>();
    // Route segment represents part (segment) of the road. 
	// In our current data it's always length of 1: [X, X + 1] or [X - 1, X] 
    
    // Route segment structure is stored separately to avoid strong reference cycles like
    // this->oppositeDirection->oppositeDirection = this
    // The structure holds next, nextLoaded, oppositeDirection, reverseSearch, parentRoute and opposite fields of the
    // corresponding Java class
    std::weak_ptr<RouteSegmentStructure> segmentStructure;
    
    // # Final fields that store objects
    uint16_t segmentStart;
    uint16_t segmentEnd;
    SHARED_PTR<RouteDataObject> road;
    // Segments only allowed for Navigation connected to the same end point
//    SHARED_PTR<RouteSegment> next;
    // # Represents cheap-storage of LinkedList connected segments
	// All the road segments from map data connected to the same end point 
//    SHARED_PTR<RouteSegment> nextLoaded;
//    SHARED_PTR<RouteSegment> oppositeDirection;
    // Same Road/ same Segment but used for opposite A* search (important to have different cause #parentRoute is different)
	// Note: if we use 1-direction A* then this is field is not needed
//    SHARED_PTR<RouteSegment> reverseSearch;
    
    // # Important for A*-search to distinguish whether segment was visited or not
	// Initially all segments null and startSegment/endSegment.parentRoute = RouteSegment::breakSegment;
	// After iteration stores previous segment i.e. how it was reached from startSegment
//    SHARED_PTR<RouteSegment> parentRoute;
    
    // final route segment
    int8_t reverseWaySearch;
    // # Caches of similar segments to speed up routing calculation 
	// Segment of opposite direction i.e. for [4 -> 5], opposite [5 -> 4]
//    SHARED_PTR<RouteSegment> opposite;
    
    // # A* routing - Distance measured in time (seconds)
	// There is a small (important!!!) difference how it's calculated for visited (parentRoute != null) and non-visited
	// NON-VISITED: time from Start [End for reverse A*] to @segStart of @this, including turn time from previous segment (@parentRoute)
	// VISITED: time from Start [End for reverse A*] to @segEnd of @this, 
	//          including turn time from previous segment (@parentRoute) and obstacle / distance time between @segStart-@segEnd on @this 
    float distanceFromStart;
    // NON-VISITED: Approximated (h(x)) time from @segStart of @this route segment to End [Start for reverse A*] 
	// VISITED: Approximated (h(x)) time from @segEnd of @this route segment to End [Start for reverse A*]
    float distanceToEnd;
    bool isFinalSegment;
    
    inline bool isReverseWaySearch() {
        return reverseWaySearch == 1;
    }
    
    inline uint16_t getSegmentStart() {
        return segmentStart;
    }

    inline uint16_t getSegmentEnd() {
        return segmentEnd;
    }
    
    inline bool isPositive() {
        return segmentEnd > segmentStart;
    }
    
    inline SHARED_PTR<RouteDataObject>& getRoad() {
        return road;
    }
    
    bool isSegmentAttachedToStart() {
        return getParentRoute() != nullptr;
    }
    
	static SHARED_PTR<RouteSegment> initRouteSegment(SHARED_PTR<RouteSegment>& th, bool positiveDirection, SHARED_PTR<RouteSegmentStructure> &segmentStructure) {
		if(th->segmentStart == 0 && !positiveDirection) {
			return SHARED_PTR<RouteSegment>();
		}
		if(th->segmentStart == th->road->getPointsLength() - 1 && positiveDirection) {
			return SHARED_PTR<RouteSegment>();
		}
		
		if (th->segmentStart == th->segmentEnd) {
			throw std::invalid_argument("segmentStart or segmentEnd");
		} else {
			if (positiveDirection == (th->segmentEnd > th->segmentStart)) {
				return th;
			} else {
				if (!th->getOppositeDirection()) {
					th->setOppositeDirection(std::make_shared<RouteSegment>(th->road, th->segmentStart,
																			th->segmentEnd > th->segmentStart ? (th->segmentStart - 1) : (th->segmentStart + 1), segmentStructure));
					th->getOppositeDirection()->setOppositeDirection(th);
				}
				return th->getOppositeDirection();
			}
		}
	}

    SHARED_PTR<RouteSegment> getParentRouteOrNull() {
        return getParentRoute() == RouteSegment::breakSegment ? nullptr : getParentRoute();
    }
    
    RouteSegment()
        : segmentStart(0)
        , segmentEnd(1)
        , road(nullptr)
        , reverseWaySearch(0)
        , distanceFromStart(0)
        , distanceToEnd(0)
        , isFinalSegment(false) {
    }
    
    RouteSegment(SHARED_PTR<RouteDataObject> road, int segmentStart, int segmentEnd, SHARED_PTR<RouteSegmentStructure> &segmentStructure)
        : segmentStructure(segmentStructure)
        , segmentStart(segmentStart)
        , segmentEnd(segmentEnd)
        , road(road)
        , reverseWaySearch(0)
        , distanceFromStart(0)
        , distanceToEnd(0)
        , isFinalSegment(false) {
    }

    RouteSegment(SHARED_PTR<RouteDataObject>& road, int segmentStart, SHARED_PTR<RouteSegmentStructure> &segmentStructure)
        : segmentStructure(segmentStructure)
        , segmentStart(segmentStart)
        , segmentEnd(segmentStart < road->getPointsLength() - 1 ? segmentStart + 1 : segmentStart - 1)
        , road(road)
        , reverseWaySearch(0)
        , distanceFromStart(0)
        , distanceToEnd(0)
        , isFinalSegment(false) {
    }
    
	~RouteSegment() {
		if (!segmentStructure.expired())
			segmentStructure.lock()->clearData(this);
	}
	
	SHARED_PTR<RouteSegment> getNext() {
		if (!segmentStructure.expired())
			return segmentStructure.lock()->getNext(this);
		return nullptr;
	}
	
	SHARED_PTR<RouteSegment> getNextLoaded() {
		if (!segmentStructure.expired())
			return segmentStructure.lock()->getNextLoaded(this);
		return nullptr;
	}
	
	SHARED_PTR<RouteSegment> getOppositeDirection() {
		if (!segmentStructure.expired())
			return segmentStructure.lock()->getOppositeDirection(this);
		return nullptr;
	}
	
	SHARED_PTR<RouteSegment> getReverseSearch() {
		if (!segmentStructure.expired())
			return segmentStructure.lock()->getReverseSearch(this);
		return nullptr;
	}
	
	SHARED_PTR<RouteSegment> getParentRoute() {
		if (!segmentStructure.expired())
			return segmentStructure.lock()->getParentRoute(this);
		return nullptr;
	}
	
	SHARED_PTR<RouteSegment> getOpposite() {
		if (!segmentStructure.expired())
			return segmentStructure.lock()->getOpposite(this);
		return nullptr;
	}
	
	void setNext(SHARED_PTR<RouteSegment> value) {
		if (!segmentStructure.expired())
			segmentStructure.lock()->setNext(this, value);
	}
	
	void setNextLoaded(SHARED_PTR<RouteSegment> value) {
		if (!segmentStructure.expired())
			segmentStructure.lock()->setNextLoaded(this, value);
	}
	
	void setOppositeDirection(SHARED_PTR<RouteSegment> value) {
		if (!segmentStructure.expired())
			segmentStructure.lock()->setOppositeDirection(this, value);
	}
	
	void setReverseSearch(SHARED_PTR<RouteSegment> value) {
		if (!segmentStructure.expired())
			segmentStructure.lock()->setReverseSearch(this, value);
	}
	
	void setParentRoute(SHARED_PTR<RouteSegment> value) {
		if (!segmentStructure.expired())
		{
			segmentStructure.lock()->setParentRoute(this, value);
		}
	}
	
	void setOpposite(SHARED_PTR<RouteSegment> value) {
		if (!segmentStructure.expired())
			segmentStructure.lock()->setOpposite(this, value);
	}
};

struct RouteSegmentPoint : RouteSegment {
	RouteSegmentPoint(SHARED_PTR<RouteDataObject>& road, int segmentStart, SHARED_PTR<RouteSegmentStructure> &segmentStructure) : RouteSegment(road, segmentStart, segmentStructure) {}
	RouteSegmentPoint(SHARED_PTR<RouteSegmentPoint>& pnt, SHARED_PTR<RouteSegmentStructure> &segmentStructure) : RouteSegment(pnt->road, pnt->segmentStart, segmentStructure),
		/*distSquare(pnt->distSquare),*/ preciseX(pnt->preciseX), preciseY(pnt->preciseY) {}
	double dist;
	int preciseX;
	int preciseY;
	vector<SHARED_PTR<RouteSegmentPoint>> others;

	LatLon getPreciseLatLon() { return LatLon(get31LatitudeY(preciseY), get31LongitudeX(preciseX)); }
};

struct FinalRouteSegment : RouteSegment {
	FinalRouteSegment(SHARED_PTR<RouteDataObject>& road, int segmentStart, int segmentEnd, SHARED_PTR<RouteSegmentStructure>& segmentStructure) : RouteSegment(road, segmentStart, segmentEnd, segmentStructure) {}
	bool reverseWaySearch;
	SHARED_PTR<RouteSegment> opposite;
};

struct GpxPoint {
	int32_t ind;
	double lat;
	double lon;
	double cumDist;
	SHARED_PTR<RouteSegmentPoint> pnt;
	vector<SHARED_PTR<RouteSegmentResult>> routeToTarget;
	vector<SHARED_PTR<RouteSegmentResult>> stepBackRoute;
	int targetInd = -1;
	bool straightLine = false;

	GpxPoint(int32_t ind, double lat, double lon, double cumDist)
		: ind(ind), lat(lat), lon(lon), cumDist(cumDist){};
	
	GpxPoint(const SHARED_PTR<GpxPoint>& p)
		: ind(p->ind), lat(p->lat), lon(p->lon), cumDist(p->cumDist){};
};

#endif /*_OSMAND_ROUTE_SEGMENT_H*/
