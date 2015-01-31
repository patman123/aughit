#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/background_segm.hpp"
#include <opencv2/core/core.hpp>
#include "opencv2/video/tracking.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <Box2D/Box2D.h>
#include <vector>
#include <algorithm>
#include <string>
#include <queue>

//Change following macros for Round 1/2 and Rectangular/Semi-Circular Paddle accordingly
//Can switch between ROUND1 and ROUND2
#define ROUND2
//Can switch between RECTANGULAR and SEMICIRCULAR
#define RECTANGULAR

using namespace std;
using namespace cv;

// Define the gravity vector.
b2Vec2 gravity(0,0);

// Do we want to let bodies sleep?
bool doSleep = true;

// Construct a world object, which will hold and simulate the rigid bodies.
b2World world(gravity, doSleep);	//Takes in two parameters
float pixel=40;	//Value of pixel defined here
double WORLDH=15;	//World Height	
double WORLDW=20;	//World Width
double THICKNESS=0.2;	//Thickness defined here
b2Fixture *_paddleFixture;	//define pointer to type b2Fixture called paddle fixture
int Threshx=0, Threshy=0;		
double XOFFSET=WORLDW/8;	
int blocktag=3;		//Block tag defined to be 3
int cornertag =-3;	//Corner tag is negative
int cornersize=5;

struct MyContact {
    b2Fixture *fixtureA;
    b2Fixture *fixtureB;
    bool operator==(const MyContact& other) const
    {
        return (fixtureA == other.fixtureA) && (fixtureB == other.fixtureB);
    }
};
 
typedef struct pt_{
	int x,y;
}pt;
 
typedef struct blob_{
 
	int min_x,max_x;
	int min_y,max_y;
	int cen_x,cen_y;
	int n_pixels;
	int ID;
}blob;
 
std::vector<MyContact>_contacts;

class ball
{
public:
	ball(double x, double y);
	~ball() {};
	b2Body* body;
	b2Fixture *_ballFixture;
	int tag;
};

class paddle
{
public:
	paddle();
	~paddle() {};
	b2Body* body;
	int tag;
};

class block
{
public:
	block();
	~block() {};
	b2Body* body;
	int tag;
};

class corner
{
public:
	corner();
	~corner() {};
	b2Body* body;
	int tag;
};

class MyContactListener : public b2ContactListener {
 
public:
    std::vector<MyContact>_contacts;
 
    MyContactListener();
    ~MyContactListener();
 
    virtual void BeginContact(b2Contact* contact);
    virtual void EndContact(b2Contact* contact);
    virtual void PreSolve(b2Contact* contact, const b2Manifold* oldManifold);    
    virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
};

ball::ball(double x, double y)
{
	tag=1;
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(x,y);
	bodyDef.userData=(void *)tag;
	body = world.CreateBody(&bodyDef);
	b2CircleShape dynamicBox;
	dynamicBox.m_radius = 0.4;
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &dynamicBox;
	fixtureDef.density = 1.0f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 1.0f;
	_ballFixture = body->CreateFixture(&fixtureDef);
};

paddle::paddle()
{
	tag=2;
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(4,WORLDH-2);
	bodyDef.userData=(void *)tag;
	body = world.CreateBody(&bodyDef);
	#ifdef SEMICIRCULAR
	b2CircleShape paddleShape;
	paddleShape.m_radius = 1.5;
	#endif
	#ifdef RECTANGULAR
	b2PolygonShape paddleShape;
	paddleShape.SetAsBox(WORLDW/12, THICKNESS*2);
	#endif
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &paddleShape;
	fixtureDef.density = 10.0f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 1;
	_paddleFixture=body->CreateFixture(&fixtureDef);
};

block::block()
{
	tag=blocktag;
	blocktag++;
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(XOFFSET, WORLDH/10);
	bodyDef.userData=(void *)tag;
	body = world.CreateBody(&bodyDef);
	b2PolygonShape blockShape;
	blockShape.SetAsBox(WORLDW/24, WORLDH/30);
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &blockShape;
	fixtureDef.density = 10.0f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 1;
	body->CreateFixture(&fixtureDef);
	XOFFSET+=WORLDW/8;
};

corner::corner()
{
	tag=cornertag;
	cornertag++;
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(0,0);
	bodyDef.userData=(void *)tag;
	body = world.CreateBody(&bodyDef);
	b2Vec2 vertices[3],vertices1[3];
	vertices[0].Set(THICKNESS , THICKNESS);
	vertices[1].Set( cornersize*THICKNESS , THICKNESS);
	vertices[2].Set(THICKNESS , cornersize*THICKNESS);
	vertices1[1].Set(WORLDW-THICKNESS , THICKNESS);
	vertices1[0].Set(WORLDW-cornersize*THICKNESS, THICKNESS);
	vertices1[2].Set(WORLDW-THICKNESS ,cornersize*THICKNESS);
	b2PolygonShape cornerShape;
	if(tag==-3)
		cornerShape.Set(vertices1, 3);
	else 
		cornerShape.Set(vertices, 3); 
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &cornerShape;
	fixtureDef.density = 10.0f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 1;
	body->CreateFixture(&fixtureDef);
}

MyContactListener::MyContactListener() : _contacts() {
}
 
MyContactListener::~MyContactListener() {
}
 
void MyContactListener::BeginContact(b2Contact* contact) {
    // We need to copy out the data because the b2Contact passed in
    // is reused.
    MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
    _contacts.push_back(myContact);
}
 
void MyContactListener::EndContact(b2Contact* contact) {
    MyContact myContact = { contact->GetFixtureA(), contact->GetFixtureB() };
    std::vector<MyContact>::iterator pos;
    pos = std::find(_contacts.begin(), _contacts.end(), myContact);
    if (pos != _contacts.end()) {
        _contacts.erase(pos);
    }
}
 
void MyContactListener::PreSolve(b2Contact* contact, 
  const b2Manifold* oldManifold) {
}
 
void MyContactListener::PostSolve(b2Contact* contact, 
  const b2ContactImpulse* impulse) {
}

paddle Player;

void mouse_callback(int event, int x, int y, int flags, void* param)
{
	//This is called every time a mouse event occurs in the window
	if (event == CV_EVENT_MOUSEMOVE) { //This is executed when the left mouse button is clicked
		//Co-ordinates of the left click are assigned to global variables and flag is set to 1
		Threshx = x;
		Threshy = y;
	}
}

b2Vec2* getPoints(b2Vec2 pos[], double angle)
{
	double w=WORLDW/5.5;
	double l=THICKNESS*2;

	b2Vec2 *newp;
	newp=(b2Vec2 *)malloc(sizeof(b2Vec2)*4);
	for(int c=0;c<4;c++){
	newp[c].x=pos[c].x*cos(angle)-pos[c].y*sin(angle);
	newp[c].y=pos[c].y*cos(angle)+pos[c].x*sin(angle);
	}	
	return newp;
}
 
int main( int argc, const char** argv )
{
	b2Body* floorBody;
	b2BodyDef floorDef;
	b2FixtureDef floorFixtureDef;
	b2PolygonShape floorBox;
	b2Fixture *_floorFixture;
	MyContactListener *_contactListener;
	_contactListener = new MyContactListener();
	world.SetContactListener(_contactListener);
	b2Vec2 position;
	ball Ball(3, 3);
	b2Vec2 force = b2Vec2(0.3,2);
	float32 timeStep = 3.0f / 30.0f;
	int32 velocityIterations = 20;
	int32 positionIterations = 5;
	block Blocks[7];
	corner Corners[2];
	bool flag = true , flag_x = true;
	char index = 'a';
	char fname[15];
	int destroyed[7];
	for(int dest=0;dest<7;dest++)destroyed[dest]=0;
	char key = 0;
	int bA=0, bB=0;
	b2Vec2 locationWorld;
	Size size(WORLDW*pixel, WORLDH*pixel);
	double angle_val = 0 ;
	vector<int> pospad;
	b2Fixture *posfix;
	b2Shape *poshape;
	b2PolygonShape* pospoly;
	b2Vec2 pos[4];
	b2Vec2 *posp;
	Point rook_points[1][4];
	int npt[]={4};
	b2Vec2 newcoord;
	double *angle;
	angle=(double *)malloc(sizeof(double));
	int bottomhitcount=0;
	double drawxoff=0;
    Mat image, gameover;
	b2Body* bodyA;
	b2Body* bodyB;
	std::vector<b2Body *>toDestroy;
	char* output="";
	//Create Floor
	floorBox.SetAsBox(WORLDW/2,THICKNESS/2);
	floorFixtureDef.shape = &floorBox;
	floorFixtureDef.restitution = 1;
	floorFixtureDef.density = 10000;
	floorFixtureDef.friction = 0;
	floorDef.position.Set(WORLDW/2, WORLDH-THICKNESS/2);
	floorDef.type=b2_staticBody;
	floorBody = world.CreateBody(&floorDef);
	_floorFixture=floorBody->CreateFixture(&floorFixtureDef);
	//Left Edge
	b2Body* leftBody;
	floorBox.SetAsBox(THICKNESS/2, WORLDH/2-THICKNESS);
	floorDef.position.Set(THICKNESS/2,WORLDH/2);
	leftBody = world.CreateBody(&floorDef);
	leftBody->CreateFixture(&floorFixtureDef);
	//Right Edge
	b2Body* rightBody;
	floorDef.position.Set(WORLDW-THICKNESS/2,WORLDH/2);
	rightBody = world.CreateBody(&floorDef);
	rightBody->CreateFixture(&floorFixtureDef);
	//Top Edge
	b2Body* topBody;
	floorBox.SetAsBox(WORLDW/2,THICKNESS/2);
	floorDef.position.Set(WORLDW/2, THICKNESS/2);
	topBody = world.CreateBody(&floorDef);
	topBody->CreateFixture(&floorFixtureDef);
	//Apply Linear Impulse to Ball
	Ball.body->ApplyLinearImpulse(force, Ball.body->GetPosition());
	force = b2Vec2(-40,-30);
	namedWindow( "Capture ", CV_WINDOW_AUTOSIZE);
	setMouseCallback("Capture ", mouse_callback, NULL);
	gameover=cvLoadImageM("gameover.jpg");
    vector<blob> blobs;
	VideoWriter outputVideo("./Video/a.avi", CV_FOURCC('M','J','P','G'), 10, size, true);
	while( key != 'q' )
	{
		if(flag_x == false)
		{
			sprintf(fname, "./Video/%c.avi" , index);
			outputVideo.open(fname, CV_FOURCC('M','J','P','G'), 10, size, true);
   	 		flag_x = true;
   	 	}	
    	if (!outputVideo.isOpened())
   		{
        	cout  << "Could not open the output video for write: " << endl;
        	return -1;
   		}
		key=waitKey(33); 
		image = Mat::zeros( size, CV_8UC3 );
		double max = 0;
		int num = -1;
		Scalar color = CV_RGB( 255, 220, 0);
		rectangle( image , Point(0 * pixel, 0 * pixel),Point(WORLDW*pixel, THICKNESS*pixel),color,-2);
		rectangle( image , Point(0 * pixel, 0 * pixel),Point(THICKNESS*pixel,WORLDH*pixel),color, -2);
		rectangle( image , Point(0 * pixel, WORLDH * pixel),Point(pixel*WORLDW, (WORLDH-THICKNESS)*pixel),color, -2);
		rectangle( image , Point(WORLDW * pixel, 0 * pixel),Point((WORLDW-THICKNESS)*pixel,WORLDH*pixel),color, -2);
		
		world.Step(timeStep, velocityIterations, positionIterations);

		position = Ball.body->GetPosition();
		circle( image , Point(position.x * pixel, position.y * pixel), 0.4*pixel , CV_RGB( 0, 255, 0 ) , -7,8,0);
		locationWorld = b2Vec2(Threshx/pixel, Threshy/pixel);
		position = Player.body->GetPosition();
		posfix=Player.body->GetFixtureList();
		poshape = posfix->GetShape();
		#ifdef RECTANGULAR
   		if(poshape->GetType() == b2Shape::e_polygon){
     		pospoly = (b2PolygonShape*)poshape;
  			 }
  		pos[0]=pospoly->GetVertex(0);
  		pos[1]=pospoly->GetVertex(1);
  		pos[2]=pospoly->GetVertex(2);
  		pos[3]=pospoly->GetVertex(3);
  		#endif
	 	 newcoord.x=Threshx;
	 	 newcoord.y=WORLDH-2;
		#ifdef RECTANGULAR
		if(key == 't')
			angle_val+=0.5;
		if(key == 'y')
			angle_val-=0.5;
		*angle=angle_val;
		posp=getPoints(pos, *angle);
  		for(int c=0;c<4;c++)
  		{
  			rook_points[0][c]=Point((position.x-posp[c].x)*pixel, (position.y-posp[c].y)*pixel);
  		}
		const Point* ppt[1]= {rook_points[0]};
		#endif
		newcoord.x=Threshx/pixel;
	 	if(newcoord.x<(WORLDW-1.7-THICKNESS)&&newcoord.x>(1.7+THICKNESS))
	 	{
	 		Player.body->SetTransform(newcoord, (float)*angle);
		}
	 	#ifdef RECTANGULAR
	 	fillPoly( image, ppt, npt, 1, CV_RGB(255,0,0) );
	 	#endif 
	 	#ifdef SEMICIRCULAR
		ellipse( image,Point(position.x * pixel, position.y * pixel),cv::Size(1.5*pixel,1.5*pixel),0,180,360,CV_RGB( 255,0, 0 ),-7,8,0);
		ellipse( image,Point(position.x * pixel, position.y * pixel-15),cv::Size(1.7*pixel,1.7*pixel),0,180,0,CV_RGB( 0,0, 0 ),-7,8,0);
		#endif
		drawxoff=0; 
		for(int bc=0;bc<7;bc++)
		{
			if(destroyed[bc]==0)

			{
				rectangle( image , Point((drawxoff+WORLDW/12)*pixel, (WORLDH/15)*pixel),Point((drawxoff+WORLDW/6)*pixel, (2*WORLDH/15)*pixel),CV_RGB(0,0,255), -2);	
			}
			else if(destroyed[bc]==2)
			{
				rectangle( image , Point((drawxoff+WORLDW/12)*pixel, (WORLDH/15)*pixel),Point((drawxoff+WORLDW/6)*pixel, (2*WORLDH/15)*pixel),CV_RGB(255,255,0), -2);
			}
			drawxoff+=WORLDW/8;
		}
		Point a[3],b[3];
		a[0]=Point(THICKNESS*pixel, (THICKNESS)*pixel);
		a[1]=Point( cornersize*THICKNESS*pixel , (THICKNESS)*pixel);
		a[2]=Point(THICKNESS*pixel , cornersize*THICKNESS*pixel);
		b[0]=Point((WORLDW-THICKNESS)*pixel , (THICKNESS)*pixel);
		b[1]=Point((WORLDW-cornersize*THICKNESS)*pixel, (THICKNESS)*pixel);
		b[2]=Point((WORLDW-THICKNESS)*pixel ,(cornersize*THICKNESS)*pixel);
		fillConvexPoly(image , a, 3, CV_RGB(255,220,0));
		fillConvexPoly(image , b, 3, CV_RGB(255,220,0));
		outputVideo << image;
		position=Ball.body->GetPosition();
		if(position.y>=(WORLDH-0.4-THICKNESS-0.005))		//The ball has hit the bottom floor
			{
				bottomhitcount++;
			}
		if(position.y < WORLDH-6)
			flag = true;
		if(position.y >= WORLDH-6 && flag==true)		//Have to fix this now
		{
		 	index++;
		 	flag_x = false;
		 	outputVideo.release();
			while(1){
			char a = waitKey(33);
				if(a==27) 
				{
					flag = 0;
					break;
				}
			}
		}
		 std::vector<b2Body *>::iterator pos2;
			    for (pos2 = toDestroy.begin(); pos2 != toDestroy.end(); ++pos2) {
			    	
			        b2Body *body = *pos2;
			        world.DestroyBody(body);
			        
	    }
	    toDestroy.clear();
		std::vector<MyContact>::iterator pos;
		for(pos = _contactListener->_contacts.begin(); 
		  pos != _contactListener->_contacts.end(); ++pos) {
		    MyContact contact = *pos;
		
	        bodyA = contact.fixtureA->GetBody();
	        bodyB = contact.fixtureB->GetBody();
			if (bodyA->GetUserData() != NULL && bodyB->GetUserData() != NULL) {
				bA=(int)bodyA->GetUserData();
				bB=(int)bodyB->GetUserData();
			            //Sprite A = ball, Sprite B = Block
			            if (bA == 1 && bB > 2) {
			            #ifdef ROUND2
			            	if(bB==4)
			            		destroyed[bB-3]=2;
			            	else
			           	#endif
			                if (std::find(toDestroy.begin(), toDestroy.end(), bodyB) == toDestroy.end()) {
			                    toDestroy.push_back(bodyB);
			                    destroyed[bB-3]=1;
			                }
			            }
			 
			            //Sprite A = block, Sprite B = ball
			            else if (bA > 2 && bB == 1) {
			            #ifdef ROUND2
			            	if(bA==4)
			            		destroyed[bA-3]=2;
			            	else
			            #endif
			                if (std::find(toDestroy.begin(), toDestroy.end(), bodyA) == toDestroy.end()) {
			                    toDestroy.push_back(bodyA);
			                    destroyed[bA-3]=1;
			                }
			            }
			        }
			    
				 }
		imshow( "Capture ", image );
	}
	resize(gameover, gameover ,size);
	imshow("Capture ", gameover);
	waitKey(2000);
}
