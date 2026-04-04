let flock = [];
let obstacles = [];
function setup(){
  stroke(255);
  fill(255);
  background(0); 
  createCanvas(1000,800);
  for(let i = 0; i < 200; i ++){
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
    stroke(255);
  for (let boid of flock){
    boid.movement();
    boid.display();

  }
}

function updateMouseObstacle(){
  obstacles.shift();
  obstacles.unshift(createVector(mouseX,mouseY));
}

class Boid{
  
  
  constructor(pos,  headingAngle){
    this.speed = 3;
    this.maxTurnSpeed = 0.5;
    this.localRadius = 100;
    this.seperationRadius = 20;
    this.localObstacleRadius = 70;

    this.cohesionWeight = 10;
    this.seperationWeight = 20;
    this.alignmentWeight = 14;
    this.obstacleAvoidanceWeight = 30;

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
  //returns -1 < x < 1 based on how magnitude and direction of angle displacement.
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
        return -(2*PI - diff)/PI;
    }else if (diff < PI){
        
       return diff/PI;
       
    }
    return 0;
  }
  movement(){
    let turn = 0;
    turn += this.cohesion()*this.cohesionWeight;
    turn += this.alignment()*this.alignmentWeight;
    turn += this.seperation()*this.seperationWeight;
    turn += this.avoidObstacles()*this.obstacleAvoidanceWeight;
    
    let totalWeight = this.cohesionWeight + this.alignmentWeight + this.seperationWeight + this.obstacleAvoidanceWeight;

    this.headingAngle += map(turn, -totalWeight,totalWeight,-this.maxTurnSpeed,this.maxTurnSpeed);

    let headingVector =  createVector(cos(this.headingAngle),sin(this.headingAngle));
    let vel = p5.Vector.mult(headingVector,this.speed);
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
    let diameter = 10;
    let arrowLength = 15;
    let arrowWidth = diameter;
    let tipOffset = createVector(cos(this.headingAngle)*arrowLength,sin(this.headingAngle)*arrowLength);
    
    ellipse(this.pos.x,this.pos.y,diameter,diameter);
    let rightOffset = createVector(cos(this.headingAngle+ PI/2)*arrowWidth/2,sin(this.headingAngle+PI/2)*arrowWidth/2);
    let leftOffset = createVector(cos(this.headingAngle- PI/2)*arrowWidth/2,sin(this.headingAngle-PI/2)*arrowWidth/2);

    triangle(this.pos.x+tipOffset.x,this.pos.y+tipOffset.y, this.pos.x+rightOffset.x,this.pos.y+rightOffset.y, this.pos.x+leftOffset.x,this.pos.y+leftOffset.y);
  }
  
}

