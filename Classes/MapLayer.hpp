//
// Created by 박성현 on 2018. 3. 31..
//

#ifndef PROJ_ANDROID_STUDIO_MAPLAYER_HPP
#define PROJ_ANDROID_STUDIO_MAPLAYER_HPP


#include <2d/CCLayer.h>
#include <set>

USING_NS_CC;

namespace AStar {

    struct Vec2i
    {

        int x, y;
        bool operator == (const Vec2i& coordinates_);

        Vec2i(int _x, int _y): x(_x), y(_y){}
        Vec2i():x(0), y(0){}

    };


    struct ANode {
    public:
        Vec2i coordinates;

        // F = G + H
        // F: 비용
        // G: 이동비용
        // H: 목적지점까지의 예상이동비용
        int G;
        int H;

        ANode *parent;

        ANode(Vec2i _coord, ANode *_parent = nullptr) {
            parent = _parent;
            coordinates = _coord;
            G = 0;
            H = 0;
        }

        int getCost() {
            return G + H;
        }

    };

    class Huristic {
    public:
        static int manhattan(Vec2i source, Vec2i target);

        static int euclidean(Vec2i source, Vec2i target);

        static int dijkstra(Vec2i source, Vec2i target);

    private:
        static Vec2i getDelta(Vec2i source, Vec2i target);

    };


    using NodeSet = std::set<ANode *>;
    using HeuristicFunction = std::function<int(Vec2i, Vec2i)>;
    using CoordinateList = std::vector<Vec2i>;

    class MapLayer : public cocos2d::LayerColor {

    public:
        CREATE_FUNC(MapLayer);

        virtual bool init() override;
        void drawMap();

        void startAStar();


    private:
        void drawBorder();

        void drawStart(int x, int y);

        void drawEnd(int x, int y);

        void drawWall(int x, int y);

        void drawObjects();

        void drawPath(CoordinateList &path);

        void drawCurrent(Vec2i);

        bool detectCollision(Vec2i coord);

        ANode* findNodeOnList(NodeSet &nodes, Vec2i coord);

        void releaseNodes(NodeSet &nodes);

    private:

        int map0[9][16] = {
                {1, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 3, 3, 3, 3, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 3, 0, 0, 0},
                {0, 0, 0, 3, 3, 3, 3, 3, 0, 3, 0, 0, 3, 0, 0, 0},
                {0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 2, 0},
                {0, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0},
        };

        int map1[9][16] = {
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 1, 0, 3, 0, 2, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        };
        
        int map2[9][16] = {
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 3, 0, 2, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        };

        int map3[9][16] = {
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        };


        int mMap[9][16] = {
                {1, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 3, 3, 3, 3, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 3, 0, 3, 0, 0, 3, 0, 0, 0},
                {0, 0, 0, 3, 3, 3, 3, 3, 0, 3, 0, 0, 3, 0, 0, 0},
                {0, 3, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 2, 0},
                {0, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0},
                {0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 3, 0, 0, 0},
        };


        static constexpr int ROUTE = 0;
        static constexpr int START = 1;
        static constexpr int END = 2;
        static constexpr int WALL = 3;

        static constexpr float SIZE = 80;

        static constexpr int LINEAR_COST = 10; // 직선이동
        static constexpr int DIAGONAL_COST = 14; // 사선이동

        CoordinateList mDirection;
        HeuristicFunction mHeuristic;

        Vec2i mStart;
        Vec2i mEnd;


    };

}
#endif //PROJ_ANDROID_STUDIO_MAPLAYER_HPP
