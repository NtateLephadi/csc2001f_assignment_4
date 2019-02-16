/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

import java.io.FileReader;
import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.Queue;
import java.util.Map;
import java.util.LinkedList;
import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.ArrayList;

// Used to signal violations of preconditions for
// various shortest path algorithms.
class GraphException extends RuntimeException
{
   /**
	 * 
	 */
   private static final long serialVersionUID = 1L;

   public GraphException( String name )
   {
      super( name );
   }
}

// Represents an edge in the graph.
class Edge
{
   public Vertex     dest;   // Second vertex in Edge
   public double     cost;   // Edge cost
   
   public Edge( Vertex d, double c )
   {
      dest = d;
      cost = c;
   }
}

// Represents an entry in the priority queue for Dijkstra's algorithm.
class Path implements Comparable<Path>
{
   public Vertex     dest;   // w
   public double     cost;   // d(w)
   
   public Path( Vertex d, double c )
   {
      dest = d;
      cost = c;
   }
   
   public int compareTo( Path rhs )
   {
      double otherCost = rhs.cost;
      
      return cost < otherCost ? -1 : cost > otherCost ? 1 : 0;
   }
}

// Represents a vertex in the graph.
class Vertex
{
   public String     name;   // Vertex name
   public List<Edge> adj;    // Adjacent vertices
   public double     dist;   // Cost
   public Vertex     prev;   // Previous vertex on shortest path
   public int        scratch;// Extra variable used in algorithm

   public Vertex( String nm )
   { 
      name = nm; 
      adj = new LinkedList<Edge>( ); 
      reset( ); 
   }

   public void reset( )
   //  { dist = Graph.INFINITY; prev = null; pos = null; scratch = 0; }    
   { dist = SimulatorOne.INFINITY; prev = null; scratch = 0; }
     
  // public PairingHeap.Position<Path> pos;  // Used for dijkstra2 (Chapter 23)
}

// Graph class: evaluate shortest paths.
//
// CONSTRUCTION: with no parameters.
//
// ******************PUBLIC OPERATIONS**********************
// void addEdge( String v, String w, double cvw )
//                              --> Add additional edge
// void printPath( String w )   --> Print path after alg is run
// void unweighted( String s )  --> Single-source unweighted
// void dijkstra( String s )    --> Single-source weighted
// void negative( String s )    --> Single-source negative weighted
// void acyclic( String s )     --> Single-source acyclic
// ******************ERRORS*********************************
// Some error checking is performed to make sure graph is ok,
// and to make sure graph satisfies properties needed by each
// algorithm.  Exceptions are thrown if errors are detected.

public class SimulatorOne
{
   public static final double INFINITY = Double.MAX_VALUE;
   private Map<String,Vertex> vertexMap = new HashMap<String,Vertex>( );
   public ArrayList<String> pathList = new ArrayList<String>( );
   public ArrayList<Double> costList = new ArrayList<Double>( );
   public ArrayList<Integer> cheapList = new ArrayList<Integer>( );
   public int numberOfPaths = 0;

   /**
    * Add a new edge to the graph.
    */
   public void addEdge( String sourceName, String destName, double cost )
   {
      Vertex v = getVertex( sourceName );
      Vertex w = getVertex( destName );
      v.adj.add( new Edge( w, cost ) );
   }

   /**
    * Driver routine to handle unreachables and print total cost.
    * It calls recursive routine to print shortest path to
    * destNode after a shortest path algorithm has run.
    */
   public void printPath( String destName )
   {
      Vertex w = vertexMap.get( destName );
      if( w.dist == INFINITY )
         System.out.println( destName + " is unreachable" );
      else
      {
         printPath( w );
      }
   }
   
    /**
    * If vertexName is not present, add it to vertexMap.
    * In either case, return the Vertex.
    */
   private Vertex getVertex( String vertexName )
   {
      Vertex v = vertexMap.get( vertexName );
      if( v == null )
      {
         v = new Vertex( vertexName );
         vertexMap.put( vertexName, v );
      }
      return v;
   }

   /**
    * Recursive routine to print shortest path to dest
    * after running shortest path algorithm. The path
    * is known to exist.
    */
   private void printPath( Vertex dest )
   {
      if( dest.prev != null )
      {
         printPath( dest.prev );
         System.out.print( " " );
      }
      System.out.print( dest.name );
   }
   
   /**
    * Initializes the vertex output info prior to running
    * any shortest path algorithm.
    */
   private void clearAll( )
   {
      for( Vertex v : vertexMap.values( ) )
         v.reset( );
   }

   /**
    * Single-source weighted shortest-path algorithm. (Dijkstra) 
    * using priority queues based on the binary heap
    */
   public void dijkstra( String startName )
   {
      PriorityQueue<Path> pq = new PriorityQueue<Path>( );
   
      Vertex start = vertexMap.get( startName );
      if( start == null )
         throw new NoSuchElementException( "Start vertex not found" );
   
      clearAll( );
      pq.add( new Path( start, 0 ) ); start.dist = 0;
      
      int nodesSeen = 0;
      while( !pq.isEmpty( ) && nodesSeen < vertexMap.size( ) )
      {
         Path vrec = pq.remove( );
         Vertex v = vrec.dest;
         if( v.scratch != 0 )  // already processed v
            continue;
             
         v.scratch = 1;
         nodesSeen++;
      
         for( Edge e : v.adj )
         {
            Vertex w = e.dest;
            double cvw = e.cost;
            
            if( cvw < 0 )
               throw new GraphException( "Graph has negative edges" );
                
            if( w.dist > ( v.dist + cvw ) )
            {
               w.dist = v.dist + cvw;
               w.prev = v;
               pq.add( new Path( w, w.dist ) );
            }
            
            if( w.dist == ( v.dist + cvw ) )
            {
               numberOfPaths++;
            }
         }
      }
   }

   /**
    * Process a request; return false if end of file.
    */
   public void print( String startName, String destName )
   {
      if ( !( startName.equals( destName ) ) )
      {
         dijkstra( startName );
         if ( getVertex( destName ).prev != null )
         {
            printPath( getVertex( destName ).prev );
            System.out.print( " " );
            dijkstra( destName );
            printPath( startName );
            System.out.println( );
         }
         else
         {
            printPath( destName );
            dijkstra( destName );
            printPath( startName );
            System.out.println( );
         }
      }
      else
         System.out.println( destName );
   }
   
   public void setCostList ( String startName, String destName )
   {
      double cost = 0;
      dijkstra( startName );
      if ( getVertex( destName ).dist != INFINITY )
         cost = getVertex( destName ).dist;
      dijkstra( destName );
      if ( getVertex( startName ).dist != INFINITY )
         cost = getVertex( startName ).dist + cost;
      costList.add( cost );
   }
   
   public double getCost ( String startName, String destName )
   {
      double cost = 0;
      dijkstra( startName );
      if ( getVertex( destName ).dist != INFINITY )
         cost = getVertex( destName ).dist;
      dijkstra( destName );
      if ( getVertex( startName ).dist != INFINITY )
         cost = getVertex( startName ).dist + cost;
      return cost;
   }
   
   public int countNumberOfCheapestPaths( String startName, String destName )
   {
      numberOfPaths = 0;
      dijkstra( startName );
      dijkstra( destName );
      return numberOfPaths;    
   }            
   /**
    * A main routine that:
    * 1. Reads a file containing edges (supplied as a command-line parameter);
    * 2. Forms the graph;
    * 3. Repeatedly prompts for two vertices and
    *    runs the shortest path algorithm.
    * The data file is a sequence of lines of the format
    *    source destination cost
    */
   public static void main( String [ ] args )
   {
      SimulatorOne g = new SimulatorOne( );
      Scanner graphFile = new Scanner( System.in );
      int numberOfNodes = graphFile.nextInt( );
      String line, temp;
      temp = graphFile.nextLine( );  
      for( int i = 0; i < numberOfNodes; i++)            
      {
         line = graphFile.nextLine( );
         StringTokenizer st = new StringTokenizer( line );
         String source  = st.nextToken( );
         while( st.hasMoreTokens( ) )
         {   
            String dest    = st.nextToken( );
            int    cost    = Integer.parseInt( st.nextToken( ) );
            g.addEdge( source, dest, cost );
         }
      }
         
      int numberOfHospitals = graphFile.nextInt( );
      temp = graphFile.nextLine( );
      line = graphFile.nextLine( );
      StringTokenizer st1 = new StringTokenizer( line );
      ArrayList<String> hospitals = new ArrayList<String>( );
      for ( int i = 0; i < numberOfHospitals; i++ )
      {
         String hospitalNode  = st1.nextToken( );
         hospitals.add( hospitalNode );
      }
         
      int numberOfVictims = graphFile.nextInt( );
      temp = graphFile.nextLine( );
      line = graphFile.nextLine( );
      StringTokenizer st2 = new StringTokenizer( line );
      ArrayList<String> victims = new ArrayList<String>( );
      for ( int i = 0; i < numberOfVictims; i ++)
      {
         String victimNode  = st2.nextToken( );
         victims.add( victimNode );
      }
         
      for ( int i = 0; i < numberOfVictims; i++)
      {  
         System.out.println( "victim " + victims.get( i ) );
         for ( int j = 0; j < numberOfHospitals; j++)
         {
            g.setCostList( hospitals.get( j ), victims.get( i ) );
         }
         Collections.sort( g.costList );
         for ( int k = 0; k < numberOfHospitals; k++ )
         {
            if ( g.costList.get( 0 ) == g.getCost ( hospitals.get( k ), victims.get( i ) ) )
            {
               System.out.println( "hospital " + hospitals.get( k ) );
               g.print( hospitals.get( k ), victims.get( i ) );
            }
         }
         g.costList.clear( );
      }
   }
}