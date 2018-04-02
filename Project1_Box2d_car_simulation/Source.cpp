#include <Windows.h>
#include <iostream>
#include <Box2D/Box2D.h>
#include <GLFW/glfw3.h>
#include <math.h>
#include <vector>
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#define M_PI 3.14159265358979323846
using namespace std;
b2Body* wheel[4];
bool dyn=true;
float inputRotation=0.0f;
GLFWwindow* window;
GLfloat start;
const int WIDTH=640;
const int HEIGHT=480;
b2World* world;
b2Body* body,*bullet;
b2Vec2 points[4];
const float M2P=60;
const float P2M=1/M2P;
b2RevoluteJointDef jointDef;
b2Joint *fljoint,*frjoint;
float m_maxForwardSpeed=100;  // 100;
float m_maxBackwardSpeed=-20; // -20;
float m_maxDriveForce=150;    // 150;
vector<b2Body*> vec;
b2Body *myRect,*right_wheel,*left_wheel,*left_rear_wheel,*right_rear_wheel;
double time,deltaTime;

enum {
      TDC_LEFT   = 0x1,
      TDC_RIGHT  = 0x2,
      TDC_UP     = 0x4,
      TDC_DOWN   = 0x8
  };
int m_controlState=0;
char key='\0'; 
		float speed =20.0;
		b2Vec2 position;
	float angle,bounce_angle,bulletx,bullety,rotation;	
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void updateFriction(b2Body* m_body);
void updateDrive(int controlState,b2Body* m_body);
void updateTurn(int controlState,b2Body* m_body);
void update(int controlState);
void addBullets();
void update_bullet(int controlstate); 

b2Body* addRect(int x,int y,int w,int h,bool dyns=true)
{
	b2BodyDef bodyDef;
	bodyDef.position.Set(x*P2M,y*P2M);
	if(dyns){
		//dyn=false;
	bodyDef.type=b2_staticBody;	
	}else{
		//dyn=true;
		bodyDef.type=b2_dynamicBody;
		bodyDef.linearDamping = 1.0;
		bodyDef.angularDamping = 1.0;
	}
	
	body=world->CreateBody(&bodyDef);
	b2PolygonShape shape;
	shape.SetAsBox(P2M*w/2,P2M*h/2,b2Vec2(0,0),0);
	b2FixtureDef fixtureDef;
	fixtureDef.shape=&shape;
	fixtureDef.density=1.0;
	//fixtureDef.friction=0.8;//step-1 try giving 0.9 they dont fly usually this is given
	//fixtureDef.restitution=0.0;//now it is like a ball it can bounce
	body->CreateFixture(&fixtureDef);
	return body;
}


void drawSquare(b2Vec2 points[],b2Vec2 center,float angle)
{
	glColor3f(1,0,0);
	glPushMatrix();
	//the translate and rotate are used so that the movement will be realistic these matrices must pushed onto stack and popped properly
	glTranslatef(center.x*M2P,center.y*M2P,0);
	glRotatef(angle*180.0/M_PI,0,0,1);	
	
		glBegin(GL_QUADS);
		for(int i=0;i<4;i++)
			glVertex2f(points[i].x*M2P,points[i].y*M2P);
		//while going from box2d to opengl convert meters to pixel box2d works in MKS(meters-kilograms-seconds) units
		glEnd();
	
	glPopMatrix();
}
void CreateBridge(b2Body* main,vector<b2Body*> vec)
{
	
			jointDef.bodyA=main;		
		
			jointDef.enableLimit = true;
			jointDef.lowerAngle =  0;
			jointDef.upperAngle =  0;
			jointDef.enableMotor = true;//default property is false
			jointDef.maxMotorTorque = 100;
			//if your are sure about the position of the objects then dont need to set "localAnchor" position directly
			//call "Initialize" method and send the arguments
			//"frjoints" and "fljoints" are used for the left wheel and backwheel whose "lowerAngle" and "upperAngle"
			//will be set later using "setLimits"
			jointDef.bodyB=vec[0];
			jointDef.Initialize(main,vec[0],vec[0]->GetWorldCenter());			
			frjoint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
			

			jointDef.bodyB=wheel[1];			
			jointDef.Initialize(main,vec[1],vec[1]->GetWorldCenter());			
			fljoint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);	

			jointDef.bodyB=wheel[2];			
			jointDef.Initialize(main,vec[2],vec[2]->GetWorldCenter());			
			frjoint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);
			
			jointDef.bodyB=vec[3];			
			jointDef.Initialize(main,vec[3],vec[3]->GetWorldCenter());			
			fljoint = (b2RevoluteJoint*)world->CreateJoint(&jointDef);			
	}



void display()
{
	
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	b2Body* tmp=world->GetBodyList();
	while(tmp){			
		for(int i=0;i<4;i++)
			points[i]=((b2PolygonShape*)tmp->GetFixtureList()->GetShape())->GetVertex(i);			
		drawSquare(points,tmp->GetWorldCenter(),tmp->GetAngle());
		
		tmp=tmp->GetNext();
	}
	for(int i=0;i<vec.size();i++){
	updateFriction(vec[i]);
	updateDrive(m_controlState,vec[i]);	
	updateTurn(m_controlState,vec[i]);
	}
	update(m_controlState);
	//update_bullet(m_controlState);
}
 void update(int controlState) {
        for (int i = 0; i < vec.size(); i++)
            updateFriction(vec[i]);
        for (int i = 0; i < vec.size(); i++)
            updateDrive(controlState,vec[i]);
		float lockAngle = 35 * DEGTORAD;
        float turnSpeedPerSec = 160 * DEGTORAD;//from lock to lock in 0.5 sec
        float turnPerTimeStep = turnSpeedPerSec / 60.0f;
        float desiredAngle = 0;
        switch ( controlState & (TDC_LEFT|TDC_RIGHT) ) {
        case TDC_LEFT:  desiredAngle = lockAngle;  break;
        case TDC_RIGHT: desiredAngle = -lockAngle; break;
        default: ;//nothing
        }
		float angleNow = ((b2RevoluteJoint*)fljoint)->GetJointAngle();//this line is used to get the current joint angle
		//if referenceAngle of joint = 0 then GetjointAngle returns 45 degrees 
		//if referenceAngle of joint = 123 then GetjointAngle returns 168 degrees
        float angleToTurn = desiredAngle - angleNow;
        angleToTurn = b2Clamp( angleToTurn, -turnPerTimeStep, turnPerTimeStep );
        float newAngle = angleNow + angleToTurn;
		((b2RevoluteJoint*)fljoint)->SetLimits(newAngle,newAngle);//setLimits sets the lowerAngle and upperAngle of the joints this one for left wheel
		((b2RevoluteJoint*)frjoint)->SetLimits(newAngle,newAngle);//this one for right wheel
    }
b2Vec2 getLateralVelocity(b2Body* m_body) {
	b2Vec2 currentRightNormal = m_body->GetWorldVector( b2Vec2(1,0) );//b2Vec2(1,0) gives the current direction thats to the right
	return b2Dot( currentRightNormal, m_body->GetLinearVelocity() ) * currentRightNormal;//canceling the lateral velocity
  }
b2Vec2 getForwardVelocity(b2Body* m_body) {
	b2Vec2 currentRightNormal = m_body->GetWorldVector( b2Vec2(0,1) );
	return b2Dot( currentRightNormal, m_body->GetLinearVelocity() ) * currentRightNormal;
}
void updateFriction(b2Body* m_body) {
        //lateral linear velocity
        float maxLateralImpulse = 2.5f;
        b2Vec2 impulse = left_wheel->GetMass() * -getLateralVelocity(m_body);
        if ( impulse.Length() > maxLateralImpulse )
            impulse *= maxLateralImpulse / impulse.Length();
        m_body->ApplyLinearImpulse( impulse, m_body->GetWorldCenter(),true );

        //angular velocity
        m_body->ApplyAngularImpulse( 0.1f * m_body->GetInertia() * -m_body->GetAngularVelocity(),true );

        //forward linear velocity
        b2Vec2 currentForwardNormal = getForwardVelocity(m_body);
        float currentForwardSpeed = currentForwardNormal.Normalize();
        float dragForceMagnitude = -2 * currentForwardSpeed;
        m_body->ApplyForce( dragForceMagnitude * currentForwardNormal, m_body->GetWorldCenter(),true );
  }
 void updateTurn(int controlState,b2Body* m_body) {
        float desiredTorque = 0;
        switch ( controlState & (TDC_LEFT|TDC_RIGHT) ) {
            case TDC_LEFT:  desiredTorque = 15;  break;
            case TDC_RIGHT: desiredTorque = -15; break;
            default: ;//nothing
        }
        m_body->ApplyTorque( desiredTorque,true );
    }
 void updateDrive(int controlState,b2Body* m_body) {
      //find desired speed
      float desiredSpeed = 0;
      switch ( controlState & (TDC_UP|TDC_DOWN) ) {
          case TDC_UP:   desiredSpeed = m_maxForwardSpeed;  break;
          case TDC_DOWN: desiredSpeed = m_maxBackwardSpeed; break;
          default: return;//do nothing
      }
	  
      //find current speed in forward direction
      b2Vec2 currentForwardNormal = m_body->GetWorldVector( b2Vec2(0,1) );
      float currentSpeed = b2Dot( getForwardVelocity(m_body), currentForwardNormal );
      
      //apply necessary force
      float force = 0;
      if ( desiredSpeed > currentSpeed )
          force = m_maxDriveForce;
      else if ( desiredSpeed < currentSpeed )
          force = -m_maxDriveForce;
      else
          return;
      m_body->ApplyForce( force * currentForwardNormal, m_body->GetWorldCenter(),true );
  }
void init()
{
	glMatrixMode(GL_PROJECTION);	
	glOrtho(0,WIDTH,HEIGHT,0,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glClearColor(0,0,0,1);
	world=new b2World(b2Vec2(0.0,0.0));
	addRect(WIDTH/2,HEIGHT-50,WIDTH,20,true);//ground creation	
	addRect(WIDTH-20,HEIGHT,WIDTH,4000,true);
	//addRect(WIDTH/2, HEIGHT + 50, 10,10,true);
	myRect=addRect(190,300,50,90,false);// drawing a rectangle at location 190,330 with width=50 and height=90 body type dynamic
	right_wheel=addRect(220,330,8,18,false);//draw right wheel at 190+w/2=215 so x=220 and y=330 so that it will not be at the end some what above the edge since it is tire
	left_wheel=addRect(160,330,8,18,false);// same applies to here also
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	//------(160,270)-|------------------------------|-(220,270)-----//
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	//------(160,330)-|------------------------------|-(220,330)-----//
	//----------------|------------------------------|---------------//
	//----------------|------------------------------|---------------//
	wheel[0]=right_wheel;
	vec.push_back(wheel[0]);
		
	wheel[1]=left_wheel;
	vec.push_back(wheel[1]);

	right_rear_wheel=addRect(220,270,8,18,false);
	left_rear_wheel=addRect(160,270,8,18,false);
	wheel[2]=right_rear_wheel;
	vec.push_back(wheel[2]);

	wheel[3]=left_rear_wheel;
	vec.push_back(wheel[3]);

	CreateBridge(myRect,vec);
	
}

int main(int argc,char **argv)
{
	glfwInit();
	window=glfwCreateWindow(WIDTH,HEIGHT,"HELLO WORLD",NULL,NULL);
	glfwMakeContextCurrent(window);
	glViewport(0,0,WIDTH,HEIGHT);	
	glfwSetKeyCallback(window,key_callback);
	init();	
	time = glfwGetTime();
	do{	
		glfwPollEvents();
		display();	
		deltaTime = time-glfwGetTime();
		time = deltaTime;
		world->Step(1.0/80.0,8,3);		
		glfwSwapBuffers(window);		
		glfwSwapInterval(5);		
	}while(!glfwWindowShouldClose(window));
	glfwTerminate();
	return 0;
}

void update_bullet(int controlState)
{
	    
		
		//float delta = 50;
		//position.x += delta * cos(myRect->GetAngle());
		//position.y += delta * sin(myRect->GetAngle());
	//position.x += myRect->GetLocalCenter().x + ((50/2)*P2M)* cos(myRect->GetAngle());
	//position.y += myRect->GetLocalCenter().y + ((90/2)*P2M)* sin(myRect->GetAngle());
		
		//position += velocity * turnPerTimeStep;
	
		//accelaration.x = 0;
		//accelaration.y = 0;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods){
	b2Vec2 newOrigin(0.0f, 0.0f);
	
	if(key== GLFW_KEY_RIGHT && action == GLFW_PRESS){
		key='a';
		}	

	if(key==GLFW_KEY_LEFT && action == GLFW_PRESS){
		key='d';
		
	}
	if(key==GLFW_KEY_DOWN && action == GLFW_PRESS){
		key='w';	
		
	}
	if(key==GLFW_KEY_UP && action == GLFW_PRESS){
		key='s';
		
	}

	
	if(key== GLFW_KEY_RIGHT && action == GLFW_RELEASE){
		key='t';
		}	

	if(key==GLFW_KEY_LEFT && action == GLFW_RELEASE){
		key='f';
		
	}
	if(key==GLFW_KEY_DOWN && action == GLFW_RELEASE){
		key='g';
		newOrigin.Set(0.0,myRect->GetPosition().y-3.0f);
		cout<<"Bodys position"<<myRect->GetPosition().y<<endl;
		
	}
	if(key==GLFW_KEY_UP && action == GLFW_RELEASE){
		key='h';	
		newOrigin.Set(0.0,myRect->GetPosition().y-3.0f);
		cout<<"Bodys position"<<myRect->GetPosition().y<<endl;
	}
	if(key==GLFW_KEY_SPACE && action==GLFW_PRESS){
		//myRect.
		cout<<"myRect->Getposition.x  ="<<myRect->GetPosition().x*M2P<<endl;
		cout<<"myRect->Getposition.y  ="<<myRect->GetPosition().y*M2P<<endl;
		position.x = myRect->GetPosition().x*M2P;
		position.y = myRect->GetPosition().y*M2P;
		bullet = addRect(position.x,position.y,3,10,false);
		bullet->SetTransform(b2Vec2(((position.x+3)*P2M),((position.y-45)*P2M)),myRect->GetAngle());
		b2Vec2 velocity = speed * b2Vec2(sin(myRect->GetAngle()),-cos(myRect->GetAngle()));
		bullet->SetLinearVelocity(velocity);
		bullet->SetFixedRotation(true);
		bullet->SetAngularVelocity(0.0f);
		bullet->SetAngularDamping(0.0f);
		angle = myRect->GetAngle()*RADTODEG;
		if(angle > 20 && angle <40){
			key = 'n';
		}
		if(angle > 40 && angle <60){
			key = 'nw';
		}
		if(angle > 60 && angle <80){
			key = 'w1';
		}
		if(angle >80 && angle <100){
			key = 'sw';
		}
		if(angle >100 && angle <120){
			key = 's1';
		}
		if(angle >120 && angle <140){
			key = 's2';
		}
		cout<<"myRect->GetAngle() =  "<<myRect->GetAngle()*RADTODEG<<endl;		
	}
switch (key) {
        case 'a' : m_controlState |= TDC_LEFT;  break;
        case 'd' : m_controlState |= TDC_RIGHT; break;
        case 'w' : m_controlState |= TDC_UP;   /* world->ShiftOrigin(newOrigin1);*/break;
		case 's' : m_controlState |= TDC_DOWN;  /*world->ShiftOrigin(newOrigin);*/break;	
			case 'n' :
			bullet->SetTransform(b2Vec2(((position.x+30)*P2M),((position.y-25)*P2M)),myRect->GetAngle());			
			break;
			case 'nw' :
				bullet->SetTransform(b2Vec2(((position.x+30)*P2M),((position.y-8)*P2M)),myRect->GetAngle());				
				break;
			case 'w1' :
				bullet->SetTransform(b2Vec2(((position.x+30)*P2M),((position.y-10)*P2M)),myRect->GetAngle());
				cout<<"position.x = "<<position.x<<endl;
				break;
			case 'sw' :
				bullet->SetTransform(b2Vec2(((position.x+40)*P2M),((position.y-0.3)*P2M)),myRect->GetAngle());
				cout<<"position.x = "<<position.x<<endl;
				break;
			case 's1' :
				bullet->SetTransform(b2Vec2(((position.x-3)*P2M),((position.y+11)*P2M)),myRect->GetAngle());
				cout<<"position.x = "<<position.x<<endl;
				break;
			case 's2' :
				bullet->SetTransform(b2Vec2(((position.x-0.3)*P2M),((position.y+2)*P2M)),myRect->GetAngle());
				cout<<"position.x = "<<position.x<<endl;
				break;

      }
switch (key) {
        case 't' : m_controlState &= ~TDC_LEFT; break;
        case 'f' : m_controlState &= ~TDC_RIGHT; break;
        case 'g' : m_controlState &= ~TDC_UP; break;
		case 'h' : m_controlState &= ~TDC_DOWN; world->ShiftOrigin(newOrigin);break;        
        }

		
}

void addBullets(){
	
}