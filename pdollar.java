import java.io.*;
import java.util.ArrayList;

class Point{
    double X;
    double Y;
    int ID;
    public Point(double x,double y,int id)
    {   X=x;
        Y=y;
        ID=id;
    }
}

class _Point_cloud
{
  public ArrayList<Point> Points;
  public String Name;
  public Point  Origin = new Point(0.0,0.0,0);

  public _Point_cloud(String name, ArrayList<Point> points, int numPoints) {

    Name = name;
    Points = points;

    Points = PRecognizer._Re_sample(Points, numPoints);
    Points = PRecognizer.Scale(Points);
    Points = PRecognizer.TranslateTo(Points, Origin);
  }
}
class _Result {
     String Name;
    public _Result(String name)
      { Name = name;}
  }
class PRecognizer {

  static int _Num_Points = 32;// number of points
  static Point  Origin = new Point(0.0,0.0,0);
  static ArrayList<_Point_cloud> _Num_Point_Clouds = new ArrayList<_Point_cloud>();

        public PRecognizer() {  }

  public _Result Recognize(ArrayList<Point> points)
  {
    _Point_cloud foundPointCloud = null;
                points = _Re_sample(points, _Num_Points);
                points = Scale(points);
                points = TranslateTo(points, Origin);

                double score = Double.POSITIVE_INFINITY;
                for( int i = 0; i < _Num_Point_Clouds.size(); i++ ) 
                {
                        double distScore = GreedyCloudMatch(points, _Num_Point_Clouds.get(i));
                        if( distScore < score ) {
                                score = distScore; 
                                foundPointCloud = _Num_Point_Clouds.get(i); 
                        }
                }
                if(((2-score)/2)<=0)
                  {
                    return new _Result("Nomatch");
                  }
                else
                  {
                    return new _Result(foundPointCloud.Name);
                  }
  }
  public int addGesture(String name, ArrayList<Point> points)
  {
    _Num_Point_Clouds.add(new _Point_cloud(name, points, _Num_Points));
    int num = 0;
    for( int i = 0; i < _Num_Point_Clouds.size(); i++ )
    {
      if( _Num_Point_Clouds.get(i).Name.equals( name) )
        num++;
    }
    return num;
  }

  private static double GreedyCloudMatch(ArrayList<Point> points, _Point_cloud pntCloud)
  {
    double e = 0.50;
    double step = Math.floor(Math.pow(points.size(), 1 - e));

    double min = Double.POSITIVE_INFINITY;
    for( int  i = 0; i < points.size(); i += step )
    {
      double d1 = CloudDistance(points, pntCloud.Points, i);
      double d2 = CloudDistance(pntCloud.Points, points, i);
      min = Math.min(min, Math.min(d1, d2)); // min3
    }
    return min;
  }

  private static double CloudDistance(ArrayList<Point> pts1, ArrayList<Point> pts2, int start)
  {

    boolean[] matched = new boolean[pts1.size()]; 
    for( int k = 0; k < pts1.size(); k++ )
      matched[k] = false;
    double sum = 0;
    int i = start;
    do
    {
      int index = -1;
      double min = Double.POSITIVE_INFINITY;
      for( int j = 0; j < matched.length; j++ )
      {
        if( !matched[j] ) {
          double d = EuclideanDistance(pts1.get(i), pts2.get(j));
          if( d < min ) {
            min = d;
            index = j;
          }
        }
      }
      matched[index] = true;
      double weight = 1 - ((i - start + pts1.size()) % pts1.size()) / pts1.size();
      sum += weight * min;
      i = (i + 1) % pts1.size();
    } while( i != start );
    return sum;
  }

  public static ArrayList<Point> _Re_sample(ArrayList<Point> points, int n)
  {
    double I = _Path_lenght(points) / (n - 1); 
    double D = 0.0;

    ArrayList<Point> newpoints = new ArrayList<Point>(); 
    newpoints.add(points.get(0));

    for( int i = 1; i < points.size(); i++ )
    {
      if( points.get(i).ID == points.get(i-1).ID )
      {
        double d = EuclideanDistance(points.get(i - 1), points.get(i));
        if ((D + d) >= I)
        {
          double qx = points.get(i - 1).X + ((I - D) / d) * (points.get(i).X - points.get(i - 1).X);
          double qy = points.get(i - 1).Y + ((I - D) / d) * (points.get(i).Y - points.get(i - 1).Y);
          Point q = new Point(qx, qy, points.get(i).ID);
          newpoints.add(q); 
          points.add(i, q); 
          D = 0.0;
        } else {
          D += d;
        }
      }
    }

    
    if( newpoints.size() == n - 1 ) 
      newpoints.add(new Point(points.get(points.size() - 1).X, points.get(points.size() - 1).Y, points.get(points.size() - 1).ID));
    return newpoints;
  }

  public static ArrayList<Point> Scale(ArrayList<Point> points)
  {
    double minX = Double.POSITIVE_INFINITY, maxX = Double.NEGATIVE_INFINITY;
    double minY = Double.POSITIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
    for( int i = 0; i < points.size(); i++ ) {
      minX = Math.min(minX, points.get(i).X);
      minY = Math.min(minY, points.get(i).Y);
      maxX = Math.max(maxX, points.get(i).X);
      maxY = Math.max(maxY, points.get(i).Y);
    }

    double size = Math.max(maxX - minX, maxY - minY);
    ArrayList<Point> newpoints = new ArrayList<Point>();

    for( int i = 0; i < points.size(); i++ ) {
      double qx = (points.get(i).X - minX) / size;
      double qy = (points.get(i).Y - minY) / size;
      newpoints.add(new Point(qx, qy, points.get(i).ID));
    }
    return newpoints;
  }

  public static ArrayList<Point> TranslateTo(ArrayList<Point> points, Point pt) 
  {
    Point c = Centroid(points);
    ArrayList<Point> newpoints = new ArrayList<Point>();
    for( int i = 0; i < points.size(); i++ ) {
      double qx = points.get(i).X + pt.X - c.X;
      double qy = points.get(i).Y + pt.Y - c.Y;
      newpoints.add(new Point(qx, qy, points.get(i).ID));
    }
    return newpoints;
  }

  private static Point Centroid(ArrayList<Point> points)
  {
    double x = 0;
    double y = 0;
    for( int i = 0; i < points.size(); i++ ) {
      x = x + points.get(i).X;
      y = y + points.get(i).Y;
    }
    x= x/points.size();
    y= y/points.size();
    return new Point(x, y, 0);
  }

 
  private static double PathDistance(ArrayList<Point> pts1, ArrayList<Point> pts2)
  {
    double d = 0.0;
    for( int i = 0; i < pts1.size(); i++ ) 
      d += EuclideanDistance(pts1.get(i), pts2.get(i));
    return d / pts1.size();
  }

  
  private static double _Path_lenght(ArrayList<Point> points)
  {
    double d = 0.0;
    for( int i = 1; i < points.size(); i++ )
    {
      if( points.get(i).ID == points.get(i-1).ID )
        d += EuclideanDistance(points.get(i - 1), points.get(i));
    }
    return d;
  }

  private static double EuclideanDistance(Point p1, Point p2)
  {
    double dx = p2.X - p1.X;
    double dy = p2.Y - p1.Y;
    return Math.pow(dx * dx + dy * dy,0.5);
  }
}
  
    public class pdollar{

        public static void HelpScreen(){
          System.out.println("Help Screen\n");
          System.out.println("pdollar -t <gesturefile>");
          System.out.println("Adds the gesture file to the list of getsure templates\nEg. pdollar -t exclamation_point.txt\n");
          System.out.println("pdollar -r");
          System.out.println("Clears the templates\nEg. pdollar -r\n");
          System.out.println("pdollar <eventstream>");
          System.out.println("Prints the name of gestures as they are recognized from the event stream\nEg. pdollar exclamation_point_eventfile.txt\n");     
        }
    
        public static void add(String args) throws FileNotFoundException, IOException{
              int id=0;
              FileReader rdr = new FileReader(args);
              FileWriter wrtr = new FileWriter("gesture_template.txt", true);
              BufferedReader br = new BufferedReader(rdr);
              BufferedWriter bw = new BufferedWriter(wrtr);
              
              bw.write("seperation");
              bw.newLine();      
              bw.write(br.readLine());
              bw.newLine();
              String s;
    
              while((s=br.readLine()) != null) {
                     if(s.equals("BEGIN")){   
                     id++;              
                         continue;
                     }
                     else if(s.equals("END")){
                        
                        continue;
                      }
    
                    else{
                         bw.write(s + "," + Integer.toString(id));
                         bw.newLine();
                     }  
              } 
              bw.write("END");
              bw.newLine();
              bw.flush();
              bw.close();
              System.out.println("Gesture file has been added");
    
        }
    
        public static void clrGTemplate() throws FileNotFoundException{
          File file  = new File("gesture_template.txt");
          if(file.exists()){
            PrintWriter wrtr = new PrintWriter(file);
            wrtr.print("");
            System.out.println("File cleared");
          }
        }
    
    
        public static void gesturepoints(PRecognizer recog_obj) throws IOException
        {
               ArrayList<Point> pnts = new ArrayList<Point>();
               FileReader rdr = new FileReader("gesture_template.txt");
            BufferedReader br = new BufferedReader(rdr);
            String s,ges_name="";
            
            while((s = br.readLine()) != null)
            {
                
                if(s.equals("seperation")) {
                    ges_name = br.readLine();
                    continue;
                }
                else if(s.equals("END"))
                {
                    recog_obj.addGesture(ges_name,pnts); 
    
                    pnts.clear(); 
                    ges_name ="";
                }
                else {   
                    String cd[] = s.split(",");
                    Point pt = new Point( Double.parseDouble(cd[0]), Double.parseDouble(cd[1]), Integer.parseInt(cd[2]));
                    pnts.add(pt); 
                }
            }
            br.close();
        }
    
    
    public static void printEventstream(String args) throws IOException{
              
              FileReader reader = new FileReader(args);
              BufferedReader br = new BufferedReader(reader);
              ArrayList<Point> pt_list = new ArrayList<Point>();
              PRecognizer recog_obj = new PRecognizer();
              gesturepoints(recog_obj);
              int num = 0;
               
               String s;
               while((s= br.readLine()) != null) {
                   if(s.equals("MOUSEDOWN")){
                       num=num+1;
                       continue;
                   }
                   else if(s.equals("MOUSEUP")){
                       
                       continue;    
                   }
                   else if(s.equals("RECOGNIZE")){
                      
                       _Result result = recog_obj.Recognize(pt_list);
                       System.out.println(result.Name);
                       pt_list.clear();
                       result = null;
    
                   }
                   else{
                       String cd[] = s.split(",");
    
                       Point pt = new Point(Double.parseDouble(cd[0]),Double.parseDouble(cd[1]),num);
                       pt_list.add(pt);
                   }    
               }
    } 
          
    public static void main(String[] args) throws IOException{
     
          if(args.length==0)
            HelpScreen();              
              
          
          else {
            if(args[0].equals("-t") && args[1] != null)
              add(args[1]);  
             
            else if(args[0].equals("-r"))
              clrGTemplate();                 
    
            else if(args[0] != null && args.length == 1)
                 printEventstream(args[0]);           
                
            else
               System.out.println("Try a valid command");
        
          } 
      }
    }
    
    
