let flock = [];
let obstacles = [];
function setup(){
  stroke(255);
  fill(255);
  background(0); 
  createCanvas(1000,800);
  for(let i = 0; i < 100; i ++){
    let randPos = createVector(random(0,width),random(0,height));
    let randAngle = random(0,2*PI);
    flock.push(new Boid(randPos, randAngle));
  }
  
  //edge obstacle walls
  let n = 100;
  for(let i = 0; i < n; i ++){
    obstacles.push(createVector(i*width/(n-1),0));
    obstacles.push(createVector(i*width/(n-1),height));
    obstacles.push(createVector(0, i*height/(n-1)));
    obstacles.push(createVector(width, i*height/(n-1)));
  }
  obstacles.unshift(createVector(mouseX,mouseY));
}

function draw(){
  background(0);
  updateMouseObstacle();
  let n = 0;
  for (let boid of flock){
    
    //track one boid
    if(n % flock.length == 2){
      stroke(0,0,255);
      strokeWeight(10);
    }else{
      strokeWeight(2);
      stroke(255);
    }
    boid.movement();
    boid.display();
    n++;
  }
}

function updateMouseObstacle(){
  obstacles.shift();
  obstacles.unshift(createVector(mouseX,mouseY));
}

class Boid{
  
  
  constructor(pos,  headingAngle){
    this.speed = 3;
    this.turnSpeed = 0.5;
    this.localRadius = 70;
    this.seperationRadius = 15;
    this.localObstacleRadius = 70;
    this.cohesionWeight = 0.05;
    this.seperationWeight = 0.1;
    this.alignmentWeight = 0.06;
    this.obstacleAvoidanceWeight = 0.1;
    this.pos = pos;
    this.headingAngle = headingAngle;
  }
  seperation(){
    
    //get the list of too close boids
    let boidsTooClose = [];
    for (let boid of flock){
      if(boid != this && this.pos.dist(boid.pos) < this.seperationRadius){
        boidsTooClose.push(boid);
      }
    }
    if(boidsTooClose.length == 0){
      return 0;
    }
    //get the center of mass of the nearby boids
    let cm = createVector(0,0);
    for (let b of boidsTooClose){
      cm.add(b.pos);
    }
    cm.div(boidsTooClose.length);
    
    //turn away from the center of mass
    let toCM = p5.Vector.sub(cm,this.pos);
    let toCMAngle = atan2(toCM.y,toCM.x); 
    let awayFromCM = toCMAngle + PI;
    return this.getTurnDirection(awayFromCM);
  }
  alignment(){
    return this.getTurnDirection(this.getAverageLocalAlignment());
  }
  cohesion(){
      let localCM = this.getLocalCenterOfMass();
      let toCM = p5.Vector.sub(localCM,this.pos);
      let toCMAngle = atan2(toCM.y,toCM.x);  
      return this.getTurnDirection(toCMAngle);

  }
  avoidObstacles(){
    
    let obCM = createVector(0,0);
    let localObstacleCount = 0;
    for (let ob of obstacles){
      if(this.pos.dist(ob) < this.localObstacleRadius){
        obCM.add(ob);
        localObstacleCount ++;
      }
    }
    if (localObstacleCount == 0){
      return 0;
    }
    obCM.div(localObstacleCount);
    let toObCM = p5.Vector.sub(obCM,this.pos);
    let awayFromObCMAngle = atan2(toObCM.y,toObCM.x)+PI;
    return this.getTurnDirection(awayFromObCMAngle);
  }
  getTurnDirection(targetAngle){
    if(this.headingAngle == targetAngle){
      return 0;
    }

    //now all angles 0 < theta < 2pi
    let diff = (targetAngle - this.headingAngle) % (2*PI);
    if(diff < 0){
      diff += (2*PI);
    }
    if(diff > PI){
        return -(2*PI - diff);
    }else if (diff < PI){
       return diff;
    }
    return 0;
  }
  movement(){
    this.headingAngle += this.cohesion()*this.turnSpeed*this.cohesionWeight;
    this.headingAngle += this.alignment()*this.turnSpeed*this.alignmentWeight;
    this.headingAngle += this.seperation()*this.turnSpeed*this.seperationWeight;
    this.headingAngle += this.avoidObstacles()*this.turnSpeed*this.obstacleAvoidanceWeight;
    let heading =  createVector(cos(this.headingAngle),sin(this.headingAngle));
    let vel = p5.Vector.mult(heading,this.speed);
    this.pos.add(vel);
    
  }
   getLocalBoids(flock, localRadius){
    let localBoids = [];
    for (let boid of flock){
      if(this.pos.dist(boid.pos) < localRadius){
        localBoids.push(boid);
      }
    }
    return localBoids;
  }
    getLocalCenterOfMass(){
    let localBoids = this.getLocalBoids(flock,this.localRadius);
    let cm = createVector(0,0);
    
    for (let b of localBoids){
      cm.add(b.pos);
    }
    
      cm.div(localBoids.length);
    return(cm);
  }
   
    getAverageLocalAlignment(){
    let localBoids = this.getLocalBoids(flock,this.localRadius);
    let summedHeadingVectors = createVector(0,0);
    for (let b of localBoids){
      summedHeadingVectors.add(createVector(cos(b.headingAngle),sin(b.headingAngle)));
    }
    let avgAngle = atan2(summedHeadingVectors.y,summedHeadingVectors.x);
    return avgAngle;
  }
  
   display(){
    let arrowLength = 23;
    let heading = createVector(cos(this.headingAngle)*arrowLength,sin(this.headingAngle)*arrowLength);
    ellipse(this.pos.x,this.pos.y,5,5);
    line(this.pos.x,this.pos.y,this.pos.x+heading.x,this.pos.y+heading.y);
  }
  
}

