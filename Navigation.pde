

float r=30;
float inf = Float.POSITIVE_INFINITY;
class NavMesh {
  int len;
  float [][] matrix;
  ArrayList<PVector> points;
  NavMesh() {
    points = new ArrayList<PVector>();
  }

  boolean checkWay(PVector a, PVector b) {
    float alpha=atan2(b.y-a.y, b.x-a.x);
    float lab = a.dist(b)+r;
    float x1, y1, x2, y2;
    x1=a.x+r*cos(alpha+HALF_PI);
    y1=a.y+r*sin(alpha+HALF_PI);
    x2=a.x+r*cos(alpha-HALF_PI);
    y2=a.y+r*sin(alpha-HALF_PI);
    PVector p0 = RayCast(a.x, a.y, alpha);
    PVector p1 = RayCast(x1, y1, alpha);
    PVector p2 = RayCast(x2, y2, alpha);

    float l0=inf, 
      l1=inf, 
      l2=inf;
    if (p0!=null)
      l0=a.dist(p0);
    if (p1!=null)
      l1=dist(p1.x, p1.y, x1, y1);
    if (p2!=null)
      l2=dist(p2.x, p2.y, x2, y2);
    return l0>=lab && l1>=lab && l2>=lab;
  }

  void createFromWalls() {
    //CreateWayPoints
    for (Polygon wall : walls) {
      int l=wall.l;
      for (int i=0; i<l; i++) {
        float a1=atan2(wall.ys[i]-wall.ys[(i+l-1)%l], 
          wall.xs[i]-wall.xs[(i+l-1)%l]);
        float a2=PI+atan2(wall.ys[(i+1)%l]-wall.ys[i], 
          wall.xs[(i+1)%l]-wall.xs[i]);
        float a=(a1+a2)/2;
        if (wall.pointIn(wall.xs[i]+cos(a), wall.ys[i]+sin(a))) {
          a+=PI;
        }
        float r1=(r+1)/sin(abs(a1-a2)/2);
        points.add(new PVector(wall.xs[i]+r1*cos(a), wall.ys[i]+r1*sin(a)));
        a+=PI;
        points.add(new PVector(wall.xs[i]+r1*cos(a), wall.ys[i]+r1*sin(a)));
      }
    }
    
    //Deleting unreacheable points
    for (int i=0; i<points.size(); i++) {
      PVector p0=points.get(i);
      if (PointIn(p0.x, p0.y)) {
        points.remove(i--);
      } else {
        boolean isReacheable=false;
        for (int j=0; j<points.size(); j++) {
          if (j!=i) {
            PVector p1=points.get(j);
            isReacheable=isReacheable || checkWay(p0, p1);
          }
        }
        if (!isReacheable) {
          points.remove(i--);
          //println("pipavs");
        }
      }
    }
    
    //Create graph
    len = points.size();
    matrix=new float[len][len];
    int i=-1, j;
    
    for(i=0;i<len;i++){
      PVector p0=points.get(i);
      for(j=i+1;j<len;j++){
        PVector p1=points.get(j);
        if (checkWay(p0, p1)) {
          matrix[i][j]=p0.dist(p1);
          matrix[j][i]=matrix[i][j];
        } else {
          matrix[i][j]=inf;
          matrix[j][i]=inf;
        }
      }
    }
    /*
    for (PVector p0 : points) {
      j=-1;
      i++;
      for (PVector p1 : points) {
        j++;
        if (p0!=p1 && checkWay(p0, p1)) {
          stroke(255-i, 0, i);
          line(p0.x, p0.y, p1.x, p1.y);
          matrix[i][j]=min(p0.dist(p1),matrix[i][j]);
          matrix[j][i]=matrix[i][j];
        } else {
          matrix[i][j]=inf;
        }
      }
    }*/
  }

  void display() {

    for (PVector p : points) {
      noStroke();
      fill(0, 255, 0);
      ellipse(p.x, p.y, 20, 20);

      strokeWeight(1);
      stroke(0, 0, 255);
      noFill();
      ellipse(p.x, p.y, 2*r, 2*r);
      if (PointIn(p.x, p.y)) {
        strokeWeight(5);
        stroke(255, 0, 0);
        ellipse(p.x, p.y, 20, 20);
      }
    }
    strokeWeight(1);
    stroke(0, 255, 0);
    for (int i=0; i<len; i++) {
      for (int j=i+1; j<len; j++) {
        PVector p0, p1;
        p0=points.get(i);
        p1=points.get(j);
        if (matrix[i][j] == p0.dist(p1))
          line(p0.x, p0.y, p1.x, p1.y);
      }
    }
  }
  
  IntList Dijkstra(int s, int t){
    int[] p=new int[len];
    boolean[] u=new boolean[len];
    float[] d=new float[len];
    for(int i=0;i<len;i++){
      d[i]=inf;
      u[i]=false;
    }
    d[s]=0;
    
    for(int cnt=0;cnt<len;cnt++){
      int minindex=0;
      float minl=inf;
      for(int i=0;i<len;i++){
        if(!u[i] && d[i]<minl){
          minindex=i;
          minl=d[i];
        }
      }
      if(minl==inf)
        break;
      u[minindex]=true;
      for(int i=0;i<len;i++){
        if(d[i]>minl+matrix[minindex][i]){
          d[i]=minl+matrix[minindex][i];
          p[i]=minindex;
        }
      }
    }
    
    IntList path=new IntList();
    path.append(t);
    for(int v=t;v!=s;v=p[v])
      path.append(v);
    path.append(s);
    path.reverse();
    return path;
  }

  ArrayList<PVector> findPath(PVector startVec, PVector endVec) {
    
    int start=0,end=0;
    for(int i=0;i<len;i++){
      if(a.dist(points.get(i))<a.dist(points.get(start)))
        start=i;
      if(b.dist(points.get(i))<b.dist(points.get(end)))
        end=i;
    }
    
    IntList path=Dijkstra(start,end);
    ArrayList<PVector> res=new ArrayList<PVector>();
    for(int i:path)
      res.add(points.get(i));
    return res;
  }
}
