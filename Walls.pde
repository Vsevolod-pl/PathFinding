
// a is radian
PVector intersect(float x0, float y0, float a0, float x1, float y1, float x2, float y2) {
  float b0 = y0 - tan(a0)*x0;
  float a1 = atan2(y2-y1, x2-x1);
  float b1 = y1-tan(a1)*x1;
  if (tan(a1) != tan(a0)) {
    float xi = (b0-b1)/(tan(a1)-tan(a0));
    float yi = tan(a0)*xi + b0;
    if (xi+0.0001>=min(x1, x2) && xi-0.0001<=max(x1, x2) //костыль
      && yi+0.0001>=min(y1, y2) && yi-0.0001<=max(y1, y2)) {
      if (dist(x0, y0, xi, yi)>dist(x0+cos(a0), y0+sin(a0), xi, yi)) {
        return new PVector(xi, yi);
      }
    }
  }/*else if (dist(x0, y0, x1, y1)<dist(x0, y0, x2, y2)){
   stroke(255,0,0);
   line(x0,y0,x1,y1);
   return new PVector(x1, y1);
   }else
   return new PVector(x2, y2);*/
  return null;
}

PVector RayCast(float x0, float y0, float a0) {
  PVector res = null, tmp;
  for (Polygon p : walls) {
    tmp=p.cast(x0, y0, a0);
    p.display();
    if (tmp!=null) {
      if (res==null) {
        res=tmp;
      } else {
        if (dist(x0, y0, res.x, res.y)>dist(x0, y0, tmp.x, tmp.y)) {
          res = tmp;
        }
      }
    }
  }
  return res;
}

boolean PointIn(float x, float y) {
  boolean res=false;
  for (Polygon p : walls)
    res = res || p.pointIn(x, y);
  return res;
}

int underLine(float x, float y, float x1, float y1, float x2, float y2) {
  int res = 0;
  if ( (x1!=x2) && (x>=min(x1, x2)) && (x<=max(x1, x2)) ) {
    float a=(y2-y1)/(x2-x1);
    float b=y1-a*x1;
    if (y>a*x+b) {
      res=1;
    }
  } else if (x1==x) {
    if (y>y1)
      res=1;
  } else if (x2==x) {
    if (y>y2)
      res=1;
  }

  return res;
}

class Polygon {
  int l;
  float[] xs, ys;
  Polygon(float[] x, float[] y) {
    xs = x;
    ys = y;
    l=x.length;
    walls.add(this);
  }

  void display() {
    stroke(0);
    strokeWeight(1);
    if (pointIn(mouseX-dx, mouseY-dy))
      fill(0, 255, 0, 200);
    else
      fill(255, 200);
    beginShape();
    for (int i=0; i<l+1; i++) {
      //ellipse(xs[i%l], ys[i%l], 20, 20);
      vertex(xs[i%l], ys[i%l]);
    }
    endShape();
  }

  // TODO change name
  PVector cast(float x0, float y0, float a0) {
    /*strokeWeight(5);
     stroke(0,0,255);
     line(x0,y0,x0+100*cos(a0),y0+100*sin(a0));*/
    PVector res = null, tmp;
    for (int i=0; i<l+1; i++) {
      //println(
      tmp = intersect(x0, y0, a0, xs[i%l], ys[i%l], xs[(i+1)%l], ys[(i+1)%l]);
      if (tmp!=null) {
        if (res==null) {
          res=tmp;
        } else {
          if (dist(x0, y0, res.x, res.y)>dist(x0, y0, tmp.x, tmp.y)) {
            res = tmp;
          }
        }
      }
    }
    return res;
  }

  boolean pointIn(float x, float y) {
    int res=0;
    for (int i=0; i<l; i++) {
      res+=underLine(x, y, xs[i%l], ys[i%l], xs[(i+1)%l], ys[(i+1)%l]);
      if (x==xs[i%l] && y==ys[i%l]) {
        return true;
      }
    }
    if (res%2==1) {
      return true;
    } else {
      return false;
    }
  }
}

Polygon load(XML xml) {
  XML[] v = xml.getChildren("vertex");
  int l=v.length;
  float[] xs = new float[l], ys = new float[l];
  for (int i=0; i<l; i++) {
    xs[i]=v[i].getFloat("x");
    ys[i]=v[i].getFloat("y");
  }
  return new Polygon(xs, ys);
}
