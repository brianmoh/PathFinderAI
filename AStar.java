//Brian Oh, Colin Lee
//A* Search for Division Cost Function using HashSet Open/Closed Lists

import java.awt.Point;
import java.util.ArrayList;
import java.util.List;
import java.util.HashMap;
import java.lang.Math;
import java.util.Collections;
import java.util.HashSet;
import java.util.Iterator;

public class MtStHelensExp_997416529_997507852 implements AIModule
{
    /// Creates the path to the goal.
    public List<Point> createPath(final TerrainMap map)
    {
        HashMap<Point, Point> CameFrom = new HashMap<Point, Point>();
        //Get start node
        final Point StartPoint = map.getStartPoint();
        //Get the goal node
        final Point GoalPoint = map.getEndPoint();
        // Holds the resulting path
        final ArrayList<Point> path = new ArrayList<Point>();
        //Creates the open list (the set of nodes to be evaluated)
        final HashSet<Point> openSet = new HashSet<Point>();
        //Creates the closed list (the set of nodes already evaluated)
        final HashSet<Point> closedSet = new HashSet<Point>();
        //Create map of f_score
        HashMap<Point, Double> g_score = new HashMap<Point, Double>();
        HashMap<Point, Double> f_score = new HashMap<Point, Double>(); 
        //Cost from start along best known path starts off as 0
        g_score.put(StartPoint, 0.0);
        //initiale f(n) as just the heuristic for the start point.
        f_score.put(StartPoint,getHeuristic(map, StartPoint, GoalPoint));
        //add beginning point to open list
        openSet.add(new Point(StartPoint));

        //while the open list is not empty
        while(!openSet.isEmpty())
        {
            //gets the point with the smallest f_score, removes it from the open_list, and adds it to the closed_list
            final Point CurrentPoint = getMinAndPop(openSet, f_score);
            //if the currentpoint is equal to the goal point then we are done!
            if(CurrentPoint.equals(GoalPoint))
            {
                Point temp = GoalPoint;
                path.add(new Point(GoalPoint));
                while(CameFrom.containsKey(temp))
                { 
                    temp = CameFrom.get(temp);
                    path.add(new Point(temp));
                }
                Collections.reverse(path);
                //return path;
                break;
            }
            closedSet.add(new Point(CurrentPoint));
            //get the neighbors of current
            final Point[] curNeighbors = map.getNeighbors(CurrentPoint);
            //for every neighbor of current point
            for(int i = 0; i < curNeighbors.length; i++)
            {
                //if neighbor in closed list just continue
                if(closedSet.contains(curNeighbors[i]))
                {
                    continue;
                }
                //length of this path;
                double t_gscore = g_score.get(CurrentPoint) + map.getCost(CurrentPoint, curNeighbors[i]);

                //if neighbor not in OpenSet we discover a new node and add it to the open list
                if(!(openSet.contains(curNeighbors[i])))
                {
                    openSet.add(new Point(curNeighbors[i]));
                }

                else if(t_gscore >= g_score.get(curNeighbors[i]))
                {
                    continue; //THIS IS NOT A BETTER PATH
                }

                //This path is the best until now. Record it.
                CameFrom.put(curNeighbors[i], CurrentPoint);
                g_score.put(curNeighbors[i], t_gscore);
                f_score.put(curNeighbors[i], g_score.get(curNeighbors[i]) + getHeuristic(map, curNeighbors[i], GoalPoint));
            }
        
        }

        // We're done!  Hand it back.
        return path;
    }

    public Point getMinAndPop(HashSet<Point> openSet, HashMap<Point, Double> f_score)
    {
        double curMinVal = Double.MAX_VALUE;
        Point min = new Point();
        
        for(Point p : openSet){
            if (curMinVal > f_score.get(p)){
                curMinVal = f_score.get(p);
                min = p;
            }           
        }
        
        openSet.remove(min);
        return min;
    }


    private double getHeuristic(final TerrainMap map, final Point pt1, final Point pt2)
    {
        double x1 = pt1.getX();
        double y1 = pt1.getY();

        double x2 = pt2.getX();
        double y2 = pt2.getY();

        double h1 = map.getTile((int)x1, (int)y1);
        double h2 = map.getTile((int)x2, (int)y2);

        double d = Math.max(Math.abs(x1 - x2),Math.abs(y1 - y2));
        
        //double d = Math.sqrt(Math.pow(x1-x2,2)+Math.pow(y1-y2, 2));
        
        //return d+(h2-h1);
         if(h2 > h1)
        {
            return d + 1.72*(h2-h1);
        }
        else if(h2 == h1)
        {
            return (double) d;
        }
        else
        {
            return Math.max(0, d + (h2-h1));
        }
    }
}
