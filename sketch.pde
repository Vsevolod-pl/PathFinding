

boolean sth=true;
PVector a,b;
float dx,dy;
NavMesh nav;
ArrayList<Polygon> walls;
void setup(){
  walls=new ArrayList<Polygon>();
  nav=new NavMesh();
  load(loadXML("map.xml"));
  for(int i=0;i<4;i++){
    float x1=random(width);
    float y1=random(height);
    float[] xs=new float[3],
    ys=new float[3];
    for(int j=0;j<3;j++){
      xs[j]=x1+random(-200,200);
      ys[j]=y1+random(-200,200);
    }
    Polygon polygon = new Polygon(xs,ys);
  }
  
  nav.createFromWalls();
  //background(0);
  
  /*if(nav.checkWay(nav.points.get(4),
  nav.points.get(1))){
    println("yes");
  }*/
}

void mousePressed(){
  if(sth)
  a=new PVector(mouseX-dx,mouseY-dy);
  else
  b=new PVector(mouseX-dx,mouseY-dy);
  sth=!sth;
}

void keyPressed(){
  setup();
}

void draw(){
  background(0);
  translate(dx,dy);
  if(mousePressed){
    dx+=mouseX-pmouseX;
    dy+=mouseY-pmouseY;
  }
  for(Polygon wall:walls)
    wall.display();
  nav.display();
  
  stroke(0,0,255);
  strokeWeight(5);
  if(b!=null){
    ArrayList<PVector> path=nav.findPath(a,b);
    for(int i=1;i<path.size();i++){
      PVector p0=path.get(i-1);
      PVector p1=path.get(i);
      line(p0.x,p0.y,p1.x,p1.y);
    }
    noStroke();
    fill(255,0,0);
    ellipse(a.x,a.y,20,20);
    ellipse(b.x,b.y,20,20);
  }
}