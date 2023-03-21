//
// Created by zhuhongwei on 2023/3/6.
//

#ifndef REMOVERT_CLUSTING_H
#define REMOVERT_CLUSTING_H

//CPP IMPLEMENTATION OF CANOPY CLUSTERING ALGORITHM

#include<bits/stdc++.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
using namespace std;
int has_name;
double t1,t2;

class Point
{

private:

    int point_id, cluster_id;
    vector<double> dimensions;
    int total_dimensions;
    string name;

public:
    Point(int point_id, vector<double> dimensions, string name)
    {
        this->point_id = point_id;
        total_dimensions = dimensions.size();

        for(int i = 0; i<total_dimensions; i++)
            this->dimensions.push_back(dimensions[i]);

        this->name = name;
        cluster_id = -1;
    }

    int getID()
    {
        return point_id;
    }

    void setCluster(int cluster_id)
    {
        this->cluster_id = cluster_id;
    }

    double getDimensions(int index)
    {
        return dimensions[index];
    }

    int getTotaldimensions()
    {
        return total_dimensions;
    }

    string getName()
    {
        return name;
    }
};

class Cluster
{
private:

    int cluster_id;
    vector<Point> points;

public:

    void addPoint(Point point)
    {
        points.push_back(point);
    }
    int getID()
    {
        return cluster_id;
    }
    void setID(int id)
    {
        this->cluster_id = id;
    }

    vector<Point> getPoints()
    {
        return points;
    }

};

class Canopy

{
private:
    vector<Point> points;
    int total_points, total_dimensions;
    // double t1,t2;
    vector<Cluster> clusters;

public:
    Canopy(int total_points,int total_dimensions,vector<Point> points)
    {
        this->total_points = total_points;
        this->total_dimensions = total_dimensions;
        // this->t1 = t1;
        // this->t2 = t2;
        this->points = points;

    }
    double findEuclidDistance(Point p1,Point p2)
    {

        double sum = 0.0;
        for (int i = 0; i < total_dimensions; ++i)
        {
            sum +=pow(p1.getDimensions(i)-p2.getDimensions(i),2.0);
        }
        return sqrt(sum);
    }

    void removePoint(Point p)
    {
        for (int i=0;i<points.size();i++)
        {
            if(p.getID() ==points[i].getID())
            {
                points.erase(points.begin() + i);
                break;
            }
        }
    }

    void go()
    {
        srand((int)time(NULL));

        int k=1;
        while(points.size()>0)
        {
            int Index = rand()%points.size();
            // cout<<Index<<endl;
            Point temp_point = points[Index];
            Cluster cluster;
            cluster.setID(k);
            //if distance is less than t1 a new cluster is formed
            for(int i = 0;i<points.size();i++)
            {
                double distance = findEuclidDistance(temp_point,points[i]);

                if(distance<t1)
                {

                    cluster.addPoint(points[i]);
                }

            }

            vector<Point> clusterPoints = cluster.getPoints();
            //In the above formed cluster if distance is less than t2 we remove it from main set
            for(int j=0;j<clusterPoints.size();j++)
            {
                Point p = clusterPoints[j];
                double distance = findEuclidDistance(temp_point,p);
                if(distance<t2)
                    removePoint(p);
            }


            clusters.push_back(cluster);
            k++;
        }

    }

    void print()
    {


        for(int i=0;i<clusters.size();i++)
        {
            vector<Point> clusterPoints;
            clusterPoints = clusters[i].getPoints();
            cout<<endl;
            cout<<"cluster "<<clusters[i].getID()<<endl;
            // cout<<endl;

            for(int j=0;j<clusterPoints.size();j++)
            {
                Point p = clusterPoints[j];

                for(int h=0;h<total_dimensions;h++)
                {
                    cout<<p.getDimensions(h)<<" ";
                }

                if(has_name)
                    cout<<p.getName();
                cout<<endl;


            }


        }

    }

};

class KMeans
{
private:
    int K; // number of clusters
    int total_values, total_points, max_iterations;
    vector<Cluster> clusters;

    // return ID of nearest center (uses euclidean distance)
    int getIDNearestCenter(Point point)
    {
        double sum = 0.0, min_dist;
        int id_cluster_center = 0;

        for(int i = 0; i < total_values; i++)
        {
            sum += pow(clusters[0].getCentralValue(i) -
                       point.getValue(i), 2.0);
        }

        min_dist = sqrt(sum);

        for(int i = 1; i < K; i++)
        {
            double dist;
            sum = 0.0;

            for(int j = 0; j < total_values; j++)
            {
                sum += pow(clusters[i].getCentralValue(j) -
                           point.getValue(j), 2.0);
            }

            dist = sqrt(sum);

            if(dist < min_dist)
            {
                min_dist = dist;
                id_cluster_center = i;
            }
        }

        return id_cluster_center;
    }

public:
    KMeans(int K, int total_points, int total_values, int max_iterations)
    {
        this->K = K;
        this->total_points = total_points;
        this->total_values = total_values;
        this->max_iterations = max_iterations;
    }

    void run(vector<Point> & points)
    {
        if(K > total_points)
            return;

        vector<int> prohibited_indexes;

        // choose K distinct values for the centers of the clusters
        for(int i = 0; i < K; i++)
        {
            while(true)
            {
                int index_point = rand() % total_points;

                if(find(prohibited_indexes.begin(), prohibited_indexes.end(),
                        index_point) == prohibited_indexes.end())
                {
                    prohibited_indexes.push_back(index_point);
                    points[index_point].setCluster(i);
                    Cluster cluster(i, points[index_point]);
                    clusters.push_back(cluster);
                    break;
                }
            }
        }

        int iter = 1;

        while(true)
        {
            bool done = true;

            // associates each point to the nearest center
            for(int i = 0; i < total_points; i++)
            {
                int id_old_cluster = points[i].getCluster();
                int id_nearest_center = getIDNearestCenter(points[i]);

                if(id_old_cluster != id_nearest_center)
                {
                    if(id_old_cluster != -1)
                        clusters[id_old_cluster].removePoint(points[i].getID());

                    points[i].setCluster(id_nearest_center);
                    clusters[id_nearest_center].addPoint(points[i]);
                    done = false;
                }
            }

            // recalculating the center of each cluster
            for(int i = 0; i < K; i++)
            {
                for(int j = 0; j < total_values; j++)
                {
                    int total_points_cluster = clusters[i].getTotalPoints();
                    double sum = 0.0;

                    if(total_points_cluster > 0)
                    {
                        for(int p = 0; p < total_points_cluster; p++)
                            sum += clusters[i].getPoint(p).getValue(j);
                        clusters[i].setCentralValue(j, sum / total_points_cluster);
                    }
                }
            }

            if(done == true || iter >= max_iterations)
            {
                cout << "Break in iteration " << iter << "\n\n";
                break;
            }

            iter++;
        }

        // shows elements of clusters
        for(int i = 0; i < K; i++)
        {
            int total_points_cluster =  clusters[i].getTotalPoints();

            cout << "Cluster " << clusters[i].getID() + 1 << endl;
            for(int j = 0; j < total_points_cluster; j++)
            {
                cout << "Point " << clusters[i].getPoint(j).getID() + 1 << ": ";
                for(int p = 0; p < total_values; p++)
                    cout << clusters[i].getPoint(j).getValue(p) << " ";

                string point_name = clusters[i].getPoint(j).getName();

                if(point_name != "")
                    cout << "- " << point_name;

                cout << endl;
            }

            cout << "Cluster values: ";

            for(int j = 0; j < total_values; j++)
                cout << clusters[i].getCentralValue(j) << " ";

            cout << "\n\n";
        }
    }
};


/// test main
/// test Canopy
int main(int arg,char*argv[])
{

    if(arg<2)
    {
        cout<<"please enter input file name as argument"<<endl;
        return 0;
    }
    FILE* input = freopen(argv[1],"r",stdin);


    int total_points, total_dimensions;


    cin >>total_points>> total_dimensions >>t1>>t2>>has_name;
    //please read readme.txt to understand inputs



    vector<Point> points;
    string point_name;

    for(int i = 0; i < total_points; i++)
    {
        vector<double> dimensions;

        for(int j = 0; j<total_dimensions;j++)
        {
            double value;
            cin >> value;
            dimensions.push_back(value);
        }

        if(has_name)
        {
            cin >> point_name;
            Point p(i, dimensions, point_name);
            points.push_back(p);
        }
        else
        {
            Point p(i, dimensions,"");
            points.push_back(p);
        }
    }
    fclose(input);

    if(t2>=t1){
        cout<<"To proceed t2 should be less than t1"<<endl;
        return 0;
    }
    Canopy canopy(total_points, total_dimensions,points);
    canopy.go();
    canopy.print();

    return 0;
}


/// test K-means
//int main(int argc, char *argv[])
//{
//    srand (time(NULL));
//
//    int total_points, total_values, K, max_iterations, has_name;
//
//    cin >> total_points >> total_values >> K >> max_iterations >> has_name;
//
//    vector<Point> points;
//    string point_name;
//
//    for(int i = 0; i < total_points; i++)
//    {
//        vector<double> values;
//
//        for(int j = 0; j < total_values; j++)
//        {
//            double value;
//            cin >> value;
//            values.push_back(value);
//        }
//
//        if(has_name)
//        {
//            cin >> point_name;
//            Point p(i, values, point_name);
//            points.push_back(p);
//        }
//        else
//        {
//            Point p(i, values);
//            points.push_back(p);
//        }
//    }
//
//    KMeans kmeans(K, total_points, total_values, max_iterations);
//    kmeans.run(points);
//
//    return 0;
//}


#endif //REMOVERT_CLUSTING_H
