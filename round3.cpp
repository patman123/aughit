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

#define ROUND2
#define RECTANGULAR

using namespace std;
using namespace cv;

RNG rng(12345);
#define DATA(A,i,j,k)  A.data[A.step*i + A.channels()*j + k]

// Define the gravity vector.
b2Vec2 gravity(0,0);

// Do we want to let bodies sleep?
bool doSleep = true;

// Construct a world object, which will hold and simulate the rigid bodies.
b2World world(gravity, doSleep);	//takes in two parameters
float pixel=40;	//value of pixel defined here
int BLOCKCOUNT=14;	//number of blocks
int CORNERCOUNT = 2; 	//number of corners
double WORLDH=15;	//world height	
double WORLDW=20;	//world width
double THICKNESS=0.2;	//thickness defined here
b2Fixture *_paddleFixture;	//define pointer to type b2Fixture called paddle fixture
int BALL=1, PADDLE=2, BLOCK=3 , CORNER=4; //define different variables for different objects
int Threshx=0, Threshy=0;		//Set Threshold X , Threshold Y to zero 
int ballX , ballY;
double XOFFSET=WORLDW/8, YOFFSET=WORLDH/10;	//Set Xoffset Y offset
double X_CORNER=WORLDW/25, Y_CORNER=WORLDH/25;	//trying out x corner , y corner
int blocktag=3;		//block tag defined to be 3
int movingblocktag =19;
int cornertag =17;	//corner tag = 50

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

class movingblock
{
	public:
	movingblock();
	~movingblock() {};
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
	#ifdef CIRCULAR
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
	bodyDef.position.Set(XOFFSET, YOFFSET);
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
	if(XOFFSET>WORLDW-0.5){
		XOFFSET=WORLDW/8;
		YOFFSET+=2*WORLDH/15;
	}
};

movingblock::movingblock()
{
	tag=movingblocktag;
	movingblocktag++;
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position.Set(XOFFSET, YOFFSET);
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
	if(XOFFSET>WORLDW-0.5){
		XOFFSET=WORLDW/8;
		YOFFSET+=2*WORLDH/15;
	}
};


corner::corner()
{
	tag=cornertag;
	cornertag++;
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(X_CORNER,Y_CORNER);
	bodyDef.userData=(void *)tag;
	body = world.CreateBody(&bodyDef);
	b2Vec2 vertices[3],vertices1[3];
	vertices[2].Set(-1 , 1);
	vertices[1].Set( 0 , 1);
	vertices[0].Set(-1 , 0);
	vertices1[1].Set(1 , 1);
	vertices1[2].Set(0, 1);
	vertices1[0].Set(1 ,0);
	b2PolygonShape cornerShape;
	if(tag>17)
		cornerShape.Set(vertices1, 3);
	else 
		cornerShape.Set(vertices, 3); 
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &cornerShape;
	fixtureDef.density = 10.0f;
	fixtureDef.friction = 0.0f;
	fixtureDef.restitution = 1;
	body->CreateFixture(&fixtureDef);
	X_CORNER = WORLDW - X_CORNER;
};

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

Mat getbox(cv::Mat img, int c1, int c2, int c3, double *a, float *Newx){
	vector<Point2f> points;
	Point2f point;
	int countj=0;
	int totcount=1;
	Mat bin=Mat::zeros( img.size(), CV_8UC1 );
	Mat color_dst=Mat::zeros( img.size(), CV_8UC3 );
	for(int i=0; i < img.rows; i++) {
		for(int j=0; j < img.cols; j++) {
			if(DATA(img,i,j,2) > c3&&DATA(img,i,j,0) <c1&&DATA(img,i,j,1) <c2){
				DATA(bin,i,j,0)  = 255;//Assigned white
				countj+=j;
				totcount++;
			}
			else{
				DATA(bin,i,j,0)  = 0;//Assigned black
			}//Assigned black
	 	}
	 }

	*Newx=countj/totcount;
	//Canny( bin, bin, 50, 200, 3 );
    // vector<Vec4i> lines;
 //    Vec4f ln;
 //    for(int i=0; i < bin.rows; i++) {
	// 	for(int j=0; j < bin.cols; j++) {
	// 		if(DATA(bin, i, j ,0)==255){
	// 			point.x=i;
	// 			point.y=j;
	// 			points.push_back(point);
	// 		}
	//  	}
	//  }
	//  ln[0]=1;
 //    // HoughLinesP( bin, lines, 1, CV_PI/90, 30, 0, 30);
 //    // for( size_t i = 0; i < lines.size(); i++ )
 //    // {
 //    //     line( color_dst, Point(lines[i][0], lines[i][1]),
 //    //         Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
 //    //     angle=(double)(lines[i][3]-lines[i][2])/(double)(lines[i][1]-lines[i][0]);
 //    //    // if(angle>maxangle)maxangle=angle;
 //    // }
 //    if(points.size()>0){
 //    	fitLine(points, ln, CV_DIST_L2, 0, 0.01, 0.01);
 //    	*a=(double)(ln[1]/ln[0]);
 //    }
 //    else *a=0;
	// line( color_dst, Point(ln[0], ln[1]),Point(ln[2], ln[3]), Scalar(0,0,255), 3, 8 );
	// //cout<<*a<<"\n";
	return bin; //Result is returned
	//return bin;
}

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
	//cout<<newp[c].x<<" "<<newp[c].y<<"\n";
	}	
	//cout<<angle<<"\n";
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
	cout << "Enter Start Position of Ball ( X , Y ):";
	cin >> ballX >> ballY;
	b2Vec2 position;
	ball Ball(ballX, ballY);
	b2Vec2 force = b2Vec2(0.3,2);
	float32 timeStep = 3.0f / 30.0f;
	int32 velocityIterations = 20;
	int32 positionIterations = 5;
	block Blocks[BLOCKCOUNT];
	corner Corners[CORNERCOUNT];
	bool flag = true , flag_x = true;
	char aa = 'a';
	char fname[5];
	int destroyed[BLOCKCOUNT];
	for(int dest=0;dest<BLOCKCOUNT;dest++)destroyed[dest]=0;
	char key = 0;
	int bA=0, bB=0;
	b2Vec2 locationWorld;
	b2MouseJoint *_mouseJoint;
	Size size(WORLDW*pixel, WORLDH*pixel);
	int c1=218, c2=46, c3=64;
	RotatedRect rRect;
	Point2f vertices[4];
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
	double drawxoff=0, drawyoff=0;
	double drawxcorner=0 , drawycorner=0;
    Mat frame,foreground,image, gameover, lineimg;
    Mat threshold_output=Mat::zeros( size, CV_8UC1 );
	b2Body* bodyA;
	b2Body* bodyB;
	int frm=0;
	KalmanFilter KF(4, 2, 0);
	std::vector<b2Body *>toDestroy;
	// intialization of KF...
	KF.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	Mat_<float> measurement(2,1); measurement.setTo(Scalar(0));
	 
	KF.statePre.at<float>(0) = WORLDW/2*pixel;
	KF.statePre.at<float>(1) = (WORLDH-2)*pixel;
	KF.statePre.at<float>(2) = 0;
	KF.statePre.at<float>(3) = 0;
	setIdentity(KF.measurementMatrix);
	setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
	setIdentity(KF.measurementNoiseCov, Scalar::all(10));
	setIdentity(KF.errorCovPost, Scalar::all(.1));


	char* output="";

	floorBox.SetAsBox(WORLDW/2,THICKNESS/2);
	floorFixtureDef.shape = &floorBox;
	floorFixtureDef.restitution = 1;
	floorFixtureDef.density = 10000;
	floorFixtureDef.friction = 0;
	floorDef.position.Set(WORLDW/2, WORLDH-THICKNESS/2);
	floorDef.type=b2_staticBody;
	floorBody = world.CreateBody(&floorDef);
	_floorFixture=floorBody->CreateFixture(&floorFixtureDef);

	b2Body* leftBody;
	floorBox.SetAsBox(THICKNESS/2, WORLDH/2-THICKNESS);
	floorDef.position.Set(THICKNESS/2,WORLDH/2);
	leftBody = world.CreateBody(&floorDef);
	leftBody->CreateFixture(&floorFixtureDef);

	b2Body* rightBody;
	floorDef.position.Set(WORLDW-THICKNESS/2,WORLDH/2);
	rightBody = world.CreateBody(&floorDef);
	rightBody->CreateFixture(&floorFixtureDef);
	
	b2Body* topBody;
	floorBox.SetAsBox(WORLDW/2,THICKNESS/2);
	floorDef.position.Set(WORLDW/2, THICKNESS/2);
	topBody = world.CreateBody(&floorDef);
	topBody->CreateFixture(&floorFixtureDef);

	Ball.body->ApplyLinearImpulse(force, Ball.body->GetPosition());
	force = b2Vec2(-40,-30);
	//Player.body->ApplyLinearImpulse(force, Player.body->GetPosition());
	
	//namedWindow( "Lines ", CV_WINDOW_AUTOSIZE);
	namedWindow( "Capture ", CV_WINDOW_AUTOSIZE);
    //namedWindow( "Camera ", CV_WINDOW_AUTOSIZE );
    setMouseCallback("Capture ", mouse_callback, NULL);
	// cvCreateTrackbar("Threshold Red","Camera ",&c1,255); 
	// cvCreateTrackbar("Threshold Green","Camera ",&c2,255); 
	// cvCreateTrackbar("Threshold Blue","Camera ",&c3,255); 
	frame=cvLoadImage("test.jpg");
	gameover=cvLoadImageM("gameover.jpg");

    vector<blob> blobs;
    VideoCapture capture(-1);
	if(!capture.isOpened()){
		return -1;
	}
	VideoWriter outputVideo("a.avi", CV_FOURCC('M','J','P','G'), 10, size, true);
    // waitKey();
	printf("%d %d\n", (int)600, (int)800);
	while( key != 'q' )
	{
		if(flag_x == false)
		{
			sprintf(fname, "%c.avi" , aa);
			outputVideo.open(fname, CV_FOURCC('M','J','P','G'), 10, size, true);
   	 		flag_x = true;
   	 	}	
    	if (!outputVideo.isOpened())
   		{
        	cout  << "Could not open the output video for write: " << endl;
        	return -1;
   		}
	 	capture >> frame;
		resize(frame, frame, size);
		//flip(frame, frame, 1);

		key=waitKey(33);
		//threshold_output=frame.clone();
		//threshold_output = getbox(frame, c1, c2, c3); 
		lineimg=getbox(frame, c1, c2, c3, angle, &newcoord.x); 
		//cout<<newcoord.x<<"\n";
   		//GetBlobs(threshold_output,blobs);
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
		//cout<<position.x<<" "<<position.y<<" ";

		locationWorld = b2Vec2(Threshx/pixel, Threshy/pixel);

  //       b2MouseJointDef md;
  //       md.bodyA = floorBody;
  //       md.bodyB = Player.body;
  //       md.target = locationWorld;
  //       md.collideConnected = true;
  //       md.maxForce = 1000.0f * Player.body->GetMass();
 
  //       _mouseJoint = (b2MouseJoint *)_world->CreateJoint(&md);
  //       Player.body->SetAwake(true);
 
		position = Player.body->GetPosition();
		posfix=Player.body->GetFixtureList();
		poshape = posfix->GetShape();
		//angle = Player.body->GetAngle();
		//cout<<angle<<"\n";
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
		measurement(0)=newcoord.x;
		measurement(1)=*angle;
		Mat prediction = KF.predict();
		Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
		Mat estimated = KF.correct(measurement);
 
		Point statePt(estimated.at<float>(0),estimated.at<float>(1));
		Point measPt(measurement(0),measurement(1));
		newcoord.x=measPt.x;

		newcoord.x=newcoord.x/pixel;
	// 	//cout<<newcoord.x<<"\n";
	// 	*angle=0;
	// 	posp=getPoints(pos, *angle);

 //  		for(int c=0;c<4;c++)
 //  		{
 //  			rook_points[0][c]=Point((position.x-posp[c].x)*pixel, (position.y-posp[c].y)*pixel);
 //  			//cout<<(position.x-posp[c].x)*pixel<<" "<< (position.y-posp[c].y)*pixel<<"\n";
 //  			//cout<<posp[c].x<<" "<<posp[c].y<<"\n";
 //  		}
 //  		// for(int c=0;c<4;c++)
 //  		// {		
 //  		// cout<<(position.x-posp[c].x)*pixel<<" "<<(position.y-posp[c].y)*pixel<<"\n";
 //  		// }
 //  		//cout<<"\n";
	// 	const Point* ppt[1]= {rook_points[0]};
		newcoord.x=Threshx/pixel;
	 	if(newcoord.x<(WORLDW-1.7-THICKNESS)&&newcoord.x>(1.7+THICKNESS)){
	 	Player.body->SetTransform(newcoord, (float)*angle);
	 }
	//  	fillPoly( image,ppt,npt, 1, CV_RGB(255,0,0) );
	 	#ifdef CIRCULAR
		ellipse( image,Point(position.x * pixel, position.y * pixel),cv::Size(1.5*pixel,1.5*pixel),0,180,360,CV_RGB( 255,0, 0 ),-7,8,0);
		ellipse( image,Point(position.x * pixel, position.y * pixel-15),cv::Size(1.7*pixel,1.7*pixel),0,180,0,CV_RGB( 0,0, 0 ),-7,8,0);
		#endif
		// waitKey();
		//cout<<position.x<<" "<<position.y<<"\n";
		// #ifdef RECTANGULAR
		// rectangle( image , Point((WORLDW/2-WORLDW/5.5)* pixel, (WORLDH-2-2*THICKNESS)* pixel),Point((WORLDW/2+WORLDW/5.5)* pixel, (WORLDH-2+2*THICKNESS) * pixel),CV_RGB( 255, 33, 127 ), -2);
		// rRect = RotatedRect(Point2f(WORLDW/2*pixel,(WORLDH-2)*pixel), Size2f(WORLDW*2*pixel/5.5, THICKNESS*2*pixel), 30);
		// rRect.points(vertices);
		// for (int i = 0; i < 4; i++)	
		//     line(image, vertices[i], vertices[(i+1)%4], CV_RGB( 255, 33, 127 ), 15);
		// #endif
		//printf("%f , %f\n", position.x*pixel,position.y*pixel);

		//position=Player.body->GetPosition();
		drawxoff=0; 
		drawyoff=0;
		drawxcorner=0; 
		drawycorner=0;
		for(int bc=0;bc<BLOCKCOUNT;bc++)
		{
			if(destroyed[bc]==0)

			{
				rectangle( image , Point((drawxoff+WORLDW/12)*pixel, (drawyoff+WORLDH/15)*pixel),Point((drawxoff+WORLDW/6)*pixel, (drawyoff+2*WORLDH/15)*pixel),CV_RGB(0,0,255), -2);	
			}
			else if(destroyed[bc]==2)
			{
				rectangle( image , Point((drawxoff+WORLDW/12)*pixel, (drawyoff+WORLDH/15)*pixel),Point((drawxoff+WORLDW/6)*pixel, (drawyoff+2*WORLDH/15)*pixel),CV_RGB(255,255,0), -2);
			}
			drawxoff+=WORLDW/8;
			if(bc%7==6){
				drawxoff=0;
				drawyoff+=2*WORLDH/15;
			}
		}
		Point a[3],b[3];
		a[0]=Point(THICKNESS*pixel , THICKNESS*pixel);
		a[1]=Point(2*pixel , THICKNESS*pixel);
		a[2]=Point(THICKNESS*pixel, 2*pixel);
		b[0]=Point((WORLDW-THICKNESS)*pixel , THICKNESS*pixel);
		b[1]=Point((WORLDW-2)*pixel , THICKNESS*pixel);
		b[2]=Point((WORLDW-THICKNESS)*pixel, 2*pixel);
		fillConvexPoly(image , a, 3, CV_RGB(255,255,255));
		fillConvexPoly(image, b , 3, CV_RGB(255,255,255));
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
		 	aa++;
		 	flag_x = false;
		 	outputVideo.release();
			while(1)
			{
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
			 			//cout<<bA<<" "<<bB<<"\n";
			            //Sprite A = ball, Sprite B = Block
			            if (bA == 1 && bB > 2) {
			            #ifdef ROUND2
			            	if(bB==5||bB==9||bB==14)
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
			            	if(bA==5||bA==9||bA==14)
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
			   
		//if(bottomhitcount>2)break;
		imshow( "Capture ", image );
		// imshow("Camera ", lineimg);
		// imshow("Camera ", threshold_output);
		Vec3b intensity;
		// for(int ht=0;ht<600;ht++)
		// {
		// 	for(int wd=0;wd<800;wd++)
		// 	{
		// 		intensity = image.at<Vec3b>(ht, wd);
		// 		// printf("%d %d %d\n", intensity.val[0], intensity.val[1], intensity.val[2]);
		// 	}
		// }
	}

	resize(gameover, gameover ,size);
	imshow("Capture ", gameover);
	waitKey(2000);
	capture.release();
}
	