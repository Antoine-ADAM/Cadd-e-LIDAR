let centimetre=2;
const vitesseLidar=5;
let repere;
let obstacles=[];
let grafPos;
let fps;
let selectButtonTime=0;
let selectButtons=[];
let lidarN;
let structures=[];
let dataLidar;
let sampleD=360;
let mesuresLidar=[];
let saveDataLidar=[];
let vecPWM;
let acN;
let acT;
let lastPositionMouse;
let timeNotSecure=-1;
const masse=50;//Masse du cadie en kg
let distanceRouesMotrice=20;//Demi-distance entre les deux roues motrice en cm
const puissanceMoteur=50;//puissance d'un seul moteur d'une roue en w (/!\Les chaines de puissance des deux roues motrice arrière s'ont strictement identique./!\)
let cadieEnergieCine=0;
const limitVitesse=0.8;//valeur en PWM
const vitesseMaxCadieMoteur=12;//en km/h
const vitesseMax=7;//en km/h
const vitesseMaxCadieMoteurCenti=vitesseMaxCadieMoteur/36;//en cm/ms
const vitesseMoteur=vitesseMax/vitesseMaxCadieMoteur;//vitesse max pour le moteur en PWM
const timeSecureMax=(masse/2*Math.pow(vitesseMax/3.6,2)/(puissanceMoteur*2*vitesseMoteur))*1250;//temps d'accélération en ms au dessus de la vitesse max du moteur accepté dans la pire situation avec majoration de 25%
//const rapportCentiMoteur=1/(vitesseMaxCadieMoteur/36)/1000;//en cm/ms
let vitesseCadie;//PWM
let timeCalcul2=-1;
let deltaPosition=10000;
function setup() {
  print(timeSecureMax);
  vecPWM=createVector(0,0);
  createCanvas(windowWidth, windowHeight);
  repere=createVector(windowWidth/2, windowHeight/2);
  grafPos=windowWidth/4;
  fps=createVector(0,0);
  frameRate(1000);
  //lastPositionMouse=createVector(mouseX,mouseY);
  lidarN=new ButtonSelect(50,200,15,"#E8605D","Lidar",5.5);
  lidarN.max=10.25;
  lidarN.min=1;
  lidarN.pa=0.25;
  centiButton=new ButtonSelect(50,250,15,"#E8605D","Centi",centimetre);
  centiButton.max=3;
  centiButton.min=0.5;
  centiButton.pa=0.125;
  grafButton=new ButtonSelect(50,300,15,"#E8605D","Graf",grafPos);
  grafButton.max=windowWidth/2;
  grafButton.min=windowWidth/10;
  grafButton.pa=10;
  angleMode(DEGREES);
  vitesseCadie=0.0;
}
function draw(){
  centimetre=centiButton.value;
  grafPos=grafButton.value;
  if(second()!=fps.y){
    fps.set(frameRate(),second());
    //print(calcul());
  }
  view();
  calcul1();
  if(saveDataLidar[1]<10000)calcul2();
  calcul3();
  secure();
}
function calcul3(){
  if(saveDataLidar[0]==270)return[1,1];
  const angleDirection=saveDataLidar[0]-270;
  const xCentreRotation=saveDataLidar[1]/(2*cos(angleDirection))/tan(-angleDirection);
  if(angleDirection>180)distanceRouesMotrice*=-1;
  let vGauche=((xCentreRotation-distanceRouesMotrice)/xCentreRotation)*vitesseCadie;
  let vDroit=((xCentreRotation+distanceRouesMotrice)/xCentreRotation)*vitesseCadie;
  acN=pow(vitesseCadie*vitesseMaxCadieMoteurCenti*1000,2)/(-xCentreRotation*10);
  if(vGauche>1){
    vGauche=1;
    vDroit=((xCentreRotation+distanceRouesMotrice*2)/xCentreRotation)*vitesseCadie;
  }
  if(vDroit>1){
    vDroit=1;
    vGauche=((xCentreRotation-distanceRouesMotrice*2)/xCentreRotation)*vitesseCadie;
  }
  vecPWM.set(vGauche,vDroit);
}
function calcul2(){
//cadieEnergieCine+=(millis()-timeCalcul2)*fgfgdfs*puissanceMoteur;
if(deltaPosition==10000){
  deltaPosition=saveDataLidar[1];
  return;
}
if(timeCalcul2==-1){
  timeCalcul2=millis();
  return;
}
//vitesseCadie-=((deltaPosition-saveDataLidar[1])/(millis()-timeCalcul2))/vitesseMaxCadieMoteurCenti;
vitesseCadie+=((saveDataLidar[1]-100)/(millis()-timeCalcul2))/vitesseMaxCadieMoteurCenti*(millis()-timeCalcul2)*0.0005;
acT=((saveDataLidar[1]-100)/(millis()-timeCalcul2))/vitesseMaxCadieMoteurCenti*15;
if(vitesseCadie<0)vitesseCadie=0;
if(vitesseCadie>1)vitesseCadie=1;
//print(((saveDataLidar[1]-100)/(millis()-timeCalcul2))/vitesseMaxCadieMoteurCenti*(millis()-timeCalcul2));
//print(vitesseCadie);
deltaPosition=saveDataLidar[1];
timeCalcul2=millis();
}
function secure(){
  //print(acT+" acn=> "+acN);
  if(vitesseCadie>limitVitesse){
    if(timeNotSecure==-1)timeNotSecure=millis();
    else{
      if(millis()-timeNotSecure>timeSecureMax){
        saveDataLidar=[];
        timeNotSecure=-1;
      }
    }
  }else timeNotSecure=-1;
}
function calcul1(){
  stroke("#04FE00");
  strokeWeight(2);
  structures=[];
  for (let i = 1; i < mesuresLidar.length; i++) {
    const element1 = mesuresLidar[i-1];
    const element2 = mesuresLidar[i];
    
    if(element1["dis"] < 195 && 2*sin(sampleD)*element1["dis"]>abs(element1["dis"]-element2["dis"])){
      //print(2*sin(1)*element1["dis"]+">"+abs(element1["dis"]-element2["dis"]));
      line(cos(element1["i"])*element1["dis"]*centimetre+repere.x,sin(element1["i"])*element1["dis"]*centimetre+repere.y,cos(element2["i"])*element2["dis"]*centimetre+repere.x,sin(element2["i"])*element2["dis"]*centimetre+repere.y);
      //print("i =>"+i);
      
      if(structures.length==0)structures.push([]);
      else{
        const elementBlock4=structures[structures.length-1][structures[structures.length-1].length-1];
        if(elementBlock4 != null && elementBlock4["i"] != element1["i"]){
          /*if(structures.length==0 || structures[structures.length-1].length<=2)structures.push([]);    ----------------/!\ Securité valeur parasite /!\------------
          else{                                                                                           obligatoire code finale arduino
            const element3=structures[structures.length-1][structures[structures.length-1].length-2];
            if(2*sin(2)*element3["dis"]<abs(element3["dis"]-element2["dis"]))structures.push([]);
          }*/
          
        structures.push([]);
       }    
      }
      structures[structures.length-1].push(element2);
    }
  }
  if(saveDataLidar.length == 5){
    // 7km/h vitesse max de diff entre la personne est le cadie en majorant entre les de prise de mesure soit environ 0.1944 cm/milliSeconde
    //il est possible faire varié cette valeur pour prendre en compte les erreurs de mesure.
    let positionChangeMax = 0.1944*(millis()-saveDataLidar[3]);
    saveDataLidar[3]=millis();
    let check=true;
    for(let i=0;i<structures.length;i++){
      const element5=structures[i];
      let listAngleLidar=[];
      let error=false;
      let perimetreStructure=0;
      for(let iy=0;iy<element5.length;iy++){
        if(element5[iy]["i"]<=180 || element5[iy]["dis"]<50 || element5[iy]["dis"]>150)error=true;
        listAngleLidar.push(element5[iy]["i"]);
        if(iy==0)continue;
        perimetreStructure+=sqrt(pow(sin(sampleD)*element5[iy-1]["dis"],2)+pow(abs(element5[iy]["dis"]-element5[iy-1]["dis"]),2));
      }
      if(error || perimetreStructure<10 || perimetreStructure>70)continue;
      let distanceMediane=200;//                                         ----------------------- problème ne fonctionne pas
      let iMediane=sort(listAngleLidar,listAngleLidar.length)[round(listAngleLidar.length/2)];
      for(let iy=0;iy<element5.length;iy++){
        if(element5[iy]["i"]==iMediane){
          distanceMediane=element5[iy]["dis"];
          break;
        }
      }
      if(sqrt(pow(cos(saveDataLidar[0])*saveDataLidar[1]-cos(iMediane)*distanceMediane,2)+pow(sin(saveDataLidar[0])*saveDataLidar[1]-sin(iMediane)*distanceMediane,2))>positionChangeMax)continue;      
      saveDataLidar=[iMediane,distanceMediane,perimetreStructure,millis(),element5];
      //print(saveDataLidar);
      check=false;
    }
    if(check)saveDataLidar=[];
  }
}
function mouseClicked(){
  if(mouseX < repere.x+60 && mouseX >repere.x && mouseY>0 && mouseY<30){
    let stat = -2;
    for (let index = 0; index < structures.length; index++) {
      const element = structures[index];
      if(element.length <= 1 || element[0]["i"] >315 || element[0]["i"]<225 || element[0]["dis"]>125 || element[0]["dis"]<75)continue;
      let error= false;
      let listAngleLidar=[element[0]["i"]];
      //let listDistanceLidar=[element[0]["dis"]];
      let perimetreStructure=0
      for(let iy=1;iy<element.length;iy++){
        if(element[iy]["i"] >315 || element[iy]["i"]<225 || element[iy]["dis"]>125 || element[iy]["dis"]<75)error=true;
        listAngleLidar.push(element[iy]["i"]);
        //listDistanceLidar.push(element[iy]["dis"]);
        perimetreStructure+=sqrt(pow(sin(sampleD)*element[iy-1]["dis"],2)+pow(abs(element[iy]["dis"]-element[iy-1]["dis"]),2));
      }
      if(error || perimetreStructure>70 || perimetreStructure<10)continue;
      if(stat!=-2){
        stat=-1;
        continue;
      }
      let distanceMediane=200;
      stat=sort(listAngleLidar,listAngleLidar.length)[round(listAngleLidar.length/2)];
      for(let iy=0;iy<element.length;iy++){
        if(element[iy]["i"]==stat){
          distanceMediane=element[iy]["dis"];
          break;
        }
      }
      saveDataLidar=[stat,distanceMediane,perimetreStructure,millis(),element];
      vitesseCadie=0;
    }
    if(stat < 0)saveDataLidar=[];
    print(saveDataLidar);
  }
}
function view(){
  background("#E8E8B7");
  strokeCap(ROUND);
  strokeWeight(1);
  stroke("red");
  fill("#FFBE4A");
  circle(repere.x,repere.y,150*centimetre);
  noStroke();
  fill("#A2E82A");
  circle(repere.x,repere.y,120*centimetre);
  noFill();
  stroke("blue");
  circle(repere.x,repere.y,100*centimetre);
  noStroke();
  fill("#FFBE4A");
  circle(repere.x,repere.y,80*centimetre);
  fill("#E8E8B7");
  stroke("red");
  circle(repere.x,repere.y,50*centimetre);
  noStroke();
  fill("#E8E8B7");
  rect(repere.x-151*centimetre,repere.y,300*centimetre+3,150*centimetre+1);
  stroke("red");
  fill("#FFAB0B");
  rect(repere.x-45*centimetre,repere.y,90*centimetre,80*centimetre);
  line(repere.x-150*centimetre,repere.y,repere.x-50*centimetre,repere.y);
  line(repere.x+150*centimetre,repere.y,repere.x+50*centimetre,repere.y);
  strokeWeight(2);  
  stroke("black");
  fill("#9168FF");
  rect(repere.x-20*centimetre, repere.y, 40*centimetre, 80*centimetre, 10, 10,2,2);
  stroke("#8BFF5C");
  strokeWeight(20);
  point(repere.x,repere.y);
  mesuresLidar=[];
  if(lidarN.value !=10.25){
    strokeWeight(2);
    stroke(color(255,0,0, 100));
    sampleD=lidarN.value/5.5;
    let obstaclesPoint=[];
    for (let index = 0; index < obstacles.length; index++) {
      obstaclesPoint.push(obstacles[index].calcul());
      obstacles[index].laser=[[],[]];
    }
    for (let i = 0; i < 360; i+=sampleD) {
      let dis=200*centimetre;
      const coef=tan(i);
      if(i>270){
        for (let index = 0; index < obstaclesPoint.length; index++) {
          const element = obstaclesPoint[index];
          const y=coef*element[0];
          if(y<0 && y>element[2] && y<element[3]){
            const disT=sqrt(pow(element[0],2)+pow(y,2));
            if(disT<dis){
              dis=disT;
              stroke(color(0,0,255));
              point(element[0]+repere.x-1,y+repere.y);
              element[4].laser[1].push(y+repere.y);
            }
          }else{
            const x=element[3]/coef;
            if(x>0 && x<element[1] && x>element[0]){
              const disT=sqrt(pow(element[3],2)+pow(x,2));
              if(disT<dis){
                dis=disT;
                stroke("#FF2BFF");
                point(x+repere.x,element[3]+repere.y+1);
                element[4].laser[0].push(x+repere.x);
              }
            }
          }
        }
      }else if(i>180){
        for (let index = 0; index < obstaclesPoint.length; index++) {
          const element = obstaclesPoint[index];
          const y =coef*element[1];
          if(y<0 && y>element[2] && y<element[3]){
            const disT=sqrt(pow(element[1],2)+pow(y,2));
            if(disT<dis){
              dis=disT;
              stroke(color(0,0,255));
              point(element[1]+repere.x+1,y+repere.y);
              element[4].laser[1].push(y+repere.y);
            }
          }else{
            const x=element[3]/coef;
             if(x<0 && x<element[1] && x>element[0]){
              const disT=sqrt(pow(element[3],2)+pow(x,2));
              if(disT<dis){
                dis=disT;
                stroke("#FF2BFF");
                point(x+repere.x,element[3]+repere.y+1);
                element[4].laser[0].push(x+repere.x);
              }
            }
          }
        }
      }else if(i>90){
        for (let index = 0; index < obstaclesPoint.length; index++) {
          const element = obstaclesPoint[index];
          const y =coef*element[1];
          if(y>0 && y<element[3] && y>element[2]){
            const disT=sqrt(pow(element[1],2)+pow(y,2));
            if(disT<dis){
              dis=disT;
              stroke(color(0,0,255));
              point(element[1]+repere.x+1,y+repere.y);
              element[4].laser[1].push(y+repere.y);
            }
          }else{
            const x =element[2]/coef;
            if(x<0 && x<element[1] && x>element[0]){
              const disT=sqrt(pow(x,2)+pow(element[2],2));
              if(disT<dis){
                dis=disT;
                stroke("#FF2BFF");
                point(x+repere.x,element[2]+repere.y-1);
                element[4].laser[0].push(x+repere.x);
              }
            }
          }
        }
      }else{
        for (let index = 0; index < obstaclesPoint.length; index++) {
          const element = obstaclesPoint[index];
          const y=coef*element[0];
          if(y>=0 && y>element[2] && y<element[3]){
            const disT=sqrt(pow(element[0],2)+pow(y,2));
            if(disT<dis){
              dis=disT;
              stroke(color(0,0,255));
              point(element[0]+repere.x-1,y+repere.y);
              element[4].laser[1].push(y+repere.y);
            }
          }else{
            const x=element[2]/coef;
            if(x>0 && x<element[1] && x>element[0]){
              const disT = sqrt(pow(x,2)+pow(element[2],2));
              if(disT<dis){
                dis=disT;
                stroke("#FF2BFF");
                point(x+repere.x,element[2]+repere.y-1);
                element[4].laser[0].push(x+repere.x);
              }
            }
          }
        }     
      }
      stroke(color(255,0,0, 100));
      line(repere.x,repere.y,repere.x+cos(i)*dis,repere.y+sin(i)*dis);
      dis/=centimetre;
      mesuresLidar.push({i,dis});
    }
  }
  //print(mesuresLidar);
  noStroke();
  fill("#EB1C29");
  if(mouseIsPressed && mouseButton==LEFT){
    let test=true;
    if(lastPositionMouse == null){
      for(let i=0;i<obstacles.length;i++){
        if(test){
          test=obstacles[i].target();
        };
        obstacles[i].display();
      }
    }else{
      lastPositionMouse.obstacle.cible();
      for(let i=0;i<obstacles.length;i++){
        obstacles[i].display();
      }
    }
  }else{
    for(let i=0;i<obstacles.length;i++){
      obstacles[i].display();
    }
  }
  //graff1
  let resultCible=createVector(acN, acT);
  fill("#8C8E7F");
  stroke("red");
  strokeWeight(2);
  circle(repere.x+grafPos,repere.y-200,100);
  noFill();
  strokeWeight(1);
  stroke("black");
  for(let i=10;i<100;i+=10){
    circle(repere.x+grafPos,repere.y-200,i);
  }
  line(repere.x+grafPos+100,repere.y-200,repere.x+grafPos-100,repere.y-200);
  line(repere.x+grafPos,repere.y-100,repere.x+grafPos,repere.y-300);
  stroke("#E85E54");
  strokeWeight(3);
  line(repere.x+grafPos,repere.y-200,repere.x+grafPos+resultCible.x*2.5,repere.y-200+resultCible.y*2.5);
  stroke("##EB753D");
  strokeWeight(3);
  point(repere.x+grafPos,repere.y-200);
  //graff3
  let resultCible2=createVector(10, 10);
  fill("#8C8E7F");
  stroke("red");
  strokeWeight(2);
  circle(repere.x+grafPos,repere.y+200,100);
  noFill();
  strokeWeight(1);
  stroke("black");
  for(let i=10;i<100;i+=10){
    circle(repere.x+grafPos,repere.y+200,i);
  }
  line(repere.x+grafPos+100,repere.y+200,repere.x+grafPos-100,repere.y+200);
  line(repere.x+grafPos,repere.y+300,repere.x+grafPos,repere.y+100);
  stroke("#E85E54");
  strokeWeight(3);
  line(repere.x+grafPos,repere.y+200,repere.x+grafPos+cos(saveDataLidar[0])*saveDataLidar[1]*0.5,repere.y+200+sin(saveDataLidar[0])*saveDataLidar[1]*0.5);
  stroke("##EB753D");
  strokeWeight(3);
  point(repere.x+grafPos,repere.y+200);
  //graff2
  fill("#8C8E7F");
  noStroke();
  rect(repere.x-150-grafPos,repere.y-125,300,275);
  strokeWeight(1);
  stroke("black");
  fill("black");
  line(repere.x-110-grafPos,repere.y-115,repere.x-110-grafPos,repere.y+115);
  textAlign(RIGHT,CENTER);
  for(let i=0;i<=100;i+=25){
    text(i+"%",repere.x-120-grafPos,-2.3*i+115+repere.y);
    line(repere.x-115-grafPos,-2.3*i+115+repere.y,repere.x+140-grafPos,-2.3*i+115+repere.y);
  }
  strokeCap(SQUARE);
  strokeWeight(30);
  stroke("#EB3110");
  line(repere.x-grafPos-25,repere.y+115,repere.x-grafPos-25,repere.y+115-230*vecPWM.x);
  stroke("#5B49FF");
  line(repere.x-grafPos+75,repere.y+115,repere.x-grafPos+75,repere.y+115-230*vecPWM.y);
  stroke("#FF05F5");
  strokeWeight(1);
  line(repere.x-115-grafPos,repere.y+115-230*limitVitesse,repere.x+140-grafPos,repere.y+115-230*limitVitesse);
  noStroke();
  textAlign(CENTER);
  fill("#EB3110");
  text("Moteur Gauche",repere.x-grafPos-25,repere.y+130);
  fill("#5B49FF");
  text("Moteur Droit",repere.x-grafPos+75,repere.y+130);
  textAlign(CENTER,CENTER);
  for(let i=0;i<selectButtons.length;i++){
    if(mouseIsPressed)selectButtons[i].test();
    selectButtons[i].draw();
  }
  textAlign(LEFT);
  fill("#28EB52");
  rect(repere.x,0,60,30);
  fill("black");
  text("Connection",repere.x,15);
  if(lastPositionMouse!=null){
    fill("#D5E6D1");
    rect(width-250,0,width,300);
    fill("black");
    text("Horizontal=>"+(lastPositionMouse.obstacle.value.x/centimetre)+" Centimetres",width-235,25);
    text("Vertical=>"+(lastPositionMouse.obstacle.value.y/centimetre)+" Centimetres",width-235,50);
    text("Distance du lidar=>"+round(sqrt(pow(lastPositionMouse.obstacle.position.y+lastPositionMouse.obstacle.value.y/2-repere.y,2)+pow(lastPositionMouse.obstacle.value.x/2+lastPositionMouse.obstacle.position.x-repere.x,2))/centimetre)+" Centimetres",width-235,75);
  }
  fill("black");
  text("FPS:"+fps.x,10,25);
  text("© Antoine ADAM",10,50);
  //text("Seconde:"+second(),10,75);
  text("X=>"+(mouseX-(repere.x-grafPos))+" Y=>"+mouseY,mouseX,mouseY +20);
}
function mousePressed(){
  if(mouseButton==CENTER){
    obstacles.push(new Obstacle());
  }
}
function mouseReleased() {
 if(mouseButton==LEFT){
    lastPositionMouse=null;
  }
}
class ButtonSelect{
  constructor(x,y,r,c,title,v){
    this.x=x;
    this.y=y;
    this.r=r;
    this.c=c;
    this.min=0;
    this.max=100;
    this.pa=1;
    this.value=v;
    this.title=title;
    this.id=selectButtons.length;
    selectButtons.push(this);
  }
  draw(){
    fill(this.c);
    circle(this.x,this.y,this.r);
    fill("black");
    text(this.title,this.x,this.y-5);
    text(this.value,this.x,this.y+5);
  }
  test(){
    if(sqrt(pow(mouseX-this.x,2)+pow(mouseY-this.y,2))<this.r && millis()-selectButtonTime>250){
      selectButtonTime=millis();
      if(keyCode == UP_ARROW && this.value<this.max){
        this.value+=this.pa;
      }else if(keyCode == DOWN_ARROW && this.value>this.min){
        this.value-=this.pa;
      }
      return this.id;
    }
    return -1;
  }
}
class SystemLastPosition{
  constructor(x,y,instanceObstacle){
    this.x=x;
    this.y=y;
    this.obstacle=instanceObstacle;
  }
}
class Obstacle{
  constructor(){
    this.position=createVector(mouseX,mouseY);
    this.value=createVector(20,20);
    this.laser=[];
  }
  display(){
    noStroke();
    rect(this.position.x,this.position.y,this.value.x,this.value.y);
    stroke("black");
    strokeWeight(5);
    point(this.position.x+this.value.x/2,this.position.y+this.value.y/2);
  }
  calcul(){
    return([this.position.x-repere.x,this.position.x+this.value.x-repere.x,this.position.y-repere.y,this.position.y+this.value.y-repere.y,this]);//[ax,ay,bx,by,cx,cy,dx,dy,this]
  }
  cible(){
    this.position.set(mouseX+lastPositionMouse.x,mouseY+lastPositionMouse.y);
    if(keyIsPressed){
      if(keyCode === UP_ARROW){
         this.value.add(0,1);
       }else if(keyCode === DOWN_ARROW && this.value.y>5){
         this.value.add(0,-1);
       } if(keyCode === LEFT_ARROW && this.value.x>5){
        this.value.add(-1,0);
      }else if(keyCode === RIGHT_ARROW){
        this.value.add(1,0);
       }
    }
  }
  target(){
    if(mouseX>this.position.x && mouseX<this.position.x+this.value.x && mouseY>this.position.y && mouseY<this.position.y+this.value.y){
      lastPositionMouse=new SystemLastPosition(this.position.x-mouseX,this.position.y-mouseY,this);
      this.cible();
      return false;
    }
    return true;
  }
}

