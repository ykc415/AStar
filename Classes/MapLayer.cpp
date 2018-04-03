//
// Created by 박성현 on 2018. 3. 31..
//

#include <2d/CCDrawNode.h>
#include <base/CCDirector.h>
#include <2d/CCSprite.h>
#include "MapLayer.hpp"
#include <chrono>




bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}



AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return {left_.x + right_.x, left_.y + right_.y};
}


bool AStar::MapLayer::init()
{

    if (!LayerColor::initWithColor(Color4B::WHITE)) {
        return false;
    }

    mDirection = {
            { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 }, // 가로세로 이동
            { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 } // 대각선 이동
    };

    mHeuristic = &Huristic::manhattan;

    std::copy(&map1[0][0], &map1[8][15], &mMap[0][0]);


    return true;
}

void AStar::MapLayer::drawMap()
{
    drawBorder();
    drawObjects();
}

void AStar::MapLayer::drawBorder()
{
    auto size = Director::getInstance()->getWinSize();
    log("winSize width : %f, height : %f", size.width, size.height);

    // 16 x 9

    // draw vertical line
    for (int x = 0; x < 16; ++x) {
        auto draw = DrawNode::create(10);
        draw->drawLine(Point(x * SIZE, 0), Point(x * SIZE, size.height), Color4F::BLACK);
        addChild(draw);
    }

    // draw horizontal line
    for (int y = 0; y < 9; ++y) {
        auto draw = DrawNode::create(10);
        draw->drawLine(Point(0, y * SIZE), Point(size.width, y * SIZE), Color4F::BLACK);
        addChild(draw);
    }
}

void AStar::MapLayer::drawStart(int x, int y)
{
    mStart.x = x;
    mStart.y = y;

    int _y = 9 - y;
    auto start = Sprite::create("blue_box.png");
    start->setPosition(Vec2(SIZE * x + SIZE/2, SIZE * _y - SIZE/2));
    start->setAnchorPoint(Vec2(0.5f, 0.5f));
    start->setOpacity(160);
    addChild(start);
}

void AStar::MapLayer::drawEnd(int x, int y)
{
    mEnd.x = x;
    mEnd.y = y;

    int _y = 9 - y;
    auto end = Sprite::create("red_box.png");
    end->setPosition(Vec2(SIZE * x + SIZE/2, SIZE * _y - SIZE/2));
    end->setAnchorPoint(Vec2(0.5f, 0.5f));
    end->setOpacity(160);
    addChild(end);
}

void AStar::MapLayer::drawWall(int x, int y)
{
    int _y = 9 - y;
    auto wall = Sprite::create("grey_box.png");
    wall->setPosition(Vec2(SIZE * x + SIZE/2, SIZE * _y - SIZE/2));
    wall->setAnchorPoint(Vec2(0.5f, 0.5f));
    wall->setOpacity(160);
    addChild(wall);
}

void AStar::MapLayer::drawObjects() {
    for(int x=0; x<16; ++x) {
        for (int y=0; y<9; ++y) {
            auto data = mMap[y][x];
            if (data == WALL) {
                drawWall(x, y);
            } else if (data == START) {
                drawStart(x, y);
            } else if (data == END) {
                drawEnd(x, y);
            }
        }
    }
}


void AStar::MapLayer::drawPath(CoordinateList& path)
{
    for (auto& vec : path) {
        if (mMap[vec.y][vec.x] != ROUTE) {
            continue;
        }
        auto box = Sprite::create("green_box.png");
        int _y = 9 - vec.y;
        box->setPosition(Vec2(SIZE * vec.x + SIZE/2, SIZE * _y - SIZE/2));
        box->setAnchorPoint(Vec2(0.5f, 0.5f));
        box->setOpacity(160);
        addChild(box);
    }

}

void AStar::MapLayer::startAStar() {


    ANode* current = nullptr;

    NodeSet openSet, closedSet; //openset = 가볼곳 , closedset = 가본곳

    openSet.insert(new ANode(mStart));

//    log("start : (%d, %d)", mStart.x, mStart.y);
//    log("end : (%d, %d)", mEnd.x, mEnd.y);

    while(!openSet.empty()) {
        current = *openSet.begin(); // 현재노드설정

        for (auto node : openSet) {
//            log("OpenSet");
//            log("(%d, %d) => %d ", node->coordinates.x, node->coordinates.y, node->getCost());
            
            if (node->getCost() <= current->getCost()) { // 가볼곳에서 가장낮은 코스트로 현재노드 이동
                current = node;
            }
        }

        drawCurrent(current->coordinates);

        if (current->coordinates == mEnd) { // 도착하면 탈출
            break;
        }

        // closedSet에 current 넣고 openSet에서 삭제
        closedSet.insert(current);
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));

        for (int i = 0; i < 8; ++i) {
            Vec2i newCoordinates(current->coordinates + mDirection[i]);

            if (detectCollision(newCoordinates) ||
                    findNodeOnList(closedSet, newCoordinates)) { // 벽이거나 , 가본곳이면 continue
                continue;
            }

            int moveCost = current->G + ((i < 4) ? LINEAR_COST : DIAGONAL_COST); // 상하좌우 이동비용 = 10 , 대각 이동비용 = 14

//            log("current (%d, %d) -> new (%d, %d) move cost: %d",
//                current->coordinates.x, current->coordinates.y,
//                newCoordinates.x, newCoordinates.y, totalCost);

            ANode* found = findNodeOnList(openSet, newCoordinates);

            if (found == nullptr) {
                found = new ANode(newCoordinates, current); // 오픈셋에 없으면 current 노드를 부모로 새노드 생성
                found->G = moveCost; // 이동비용 설정
                found->H = mHeuristic(found->coordinates, mEnd); // 휴리스틱
                openSet.insert(found);
            }
            else if (moveCost < found->G) {  // 오픈셋에 이미 있는데 이동비용이 더작으면
                found->parent = current; // 부모를 current 로 바꿔주고 코스트변경
                found->G = moveCost;
            }
        }
    }


    // Get Result
    CoordinateList resultPath;
//    log("result");
    while (current != nullptr) {
//        log("(%d, %d)", current->coordinates.x, current->coordinates.y);
        resultPath.push_back(current->coordinates);
        current = current->parent;
    }






    releaseNodes(openSet);
    releaseNodes(closedSet);

    drawPath(resultPath);


}


bool AStar::MapLayer::detectCollision(Vec2i coord) {
    if (coord.x < 0 || coord.x > 15
        ||coord.y < 0 || coord.y > 8 ||
            mMap[(int)coord.y][(int)coord.x] == WALL) {
        
        return true;
    }
    return false;
}

AStar::ANode* AStar::MapLayer::findNodeOnList(NodeSet& nodes, Vec2i coord) {
    for (auto node : nodes) {
        if (node->coordinates == coord) {
            return node;
        }
    }
    return nullptr;
}

void AStar::MapLayer::releaseNodes(NodeSet& nodes) {
    for (auto it = nodes.begin(); it != nodes.end();) {
        delete *it;
        it = nodes.erase(it);

    }
}

void AStar::MapLayer::drawCurrent(Vec2i vec) {

    auto box = Sprite::create("dot_box.png");
    int _y = 9 - vec.y;
    box->setPosition(Vec2(SIZE * vec.x + SIZE / 2, SIZE * _y - SIZE / 2));
    box->setAnchorPoint(Vec2(0.5f, 0.5f));
    box->setOpacity(160);
    addChild(box);
}


AStar::Vec2i AStar::Huristic::getDelta(Vec2i source, Vec2i target) {
    return {abs(source.x - target.x), abs(source.y - target.y)};
}


int AStar::Huristic::euclidean(Vec2i source, Vec2i target) {
    auto delta = std::move(getDelta(source, target));
    return static_cast<int>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

int AStar::Huristic::manhattan(Vec2i source, Vec2i target) {
    auto delta = std::move(getDelta(source, target));

    return (delta.x + delta.y) * 10;
}

int AStar::Huristic::dijkstra(Vec2i source, Vec2i target) {
    return 0;
}
