#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <list>
#include <string>
#include <route_planner/KDTree.hpp>


struct Point
{
	double x, y; //点坐标
	double F, G, H; //F=G+H
	Point *parent; //parent的坐标，这里没有用指针，从而简化代码
	Point(double _x, double _y) :x(_x), y(_y), F(0.0), G(0.0), H(0.0), parent(NULL)  //变量初始化
	{
	}
};

class Astar
{
public:
  Astar() = default;
  Astar(const double &_rad,const pointVec &waypts):searching_rad(_rad),waypoints(waypts){} // use init func of kdtree
void InitAstar(const std::string &filename);
std::list<Point *> GetPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);

public:
  KDTree waypoints;
  double searching_rad = 0.2;

private:
	Point *findPath(Point &startPoint, Point &endPoint, bool isIgnoreCorner);
	std::vector<Point *> getSurroundPoints(const Point *point, bool isIgnoreCorner) const;
	bool isCanreach(const Point *point, const Point *target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断
	Point *isInList(const std::list<Point *> &list, const Point *point) const; //判断开启/关闭列表中是否包含某点
	Point *getLeastFpoint(); //从开启列表中返回F值最小的节点
	//计算FGH值
	double calcG(Point *temp_start, Point *point);
	double calcH(Point *point, Point *end);
	double calcF(Point *point);
private:
	std::list<Point *> openList;  //开启列表
	std::list<Point *> closeList; //关闭列表
};

#endif
