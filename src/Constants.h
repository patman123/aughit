/* Game Parameters*/
float pixel=40;	//Scaling Factor
int BLOCKCOUNT=7;	//Number of Static blocks
/* Game Dimensions */
double WORLDH=15;	//World height	
double WORLDW=20;	//World width
double THICKNESS=0.2;	//Thickness defined here
double XOFFSET=WORLDW/8, YOFFSET=WORLDH/10;	//Set X-offset Y-offset
/* Corners */
int CORNERCOUNT = 2; 	//Number of corners
double X_CORNER=WORLDW/25, Y_CORNER=WORLDH/25;	//Trying out X corner , Y corner
/* IDs + Threshold Values */
int BALL=1, PADDLE=2, BLOCK=3 , CORNER=4; 
int Threshx=0, Threshy=0;		//Set Threshold X , Threshold Y to zero 
int blocktag=3;		//Block ID
int cornertag =-3;	//Corner ID
int cornersize=5; 	//Size of Corner
/* Shi-Tomasi Corner Detection */
char* source_window = "Camera";
int maxCorners = 4;
int maxTrackbar = 100;
/* Noise Reduction : Morphological Opening Parameters */
int morph_elem = 1;
int morph_size = 3;
/* Variables for Round 3 */
int MBLOCKCOUNT=3;
int movingblocktag =10;
double movvel=0.5;

// Define the gravity vector.
b2Vec2 gravity(0,0);
// Do we want to let bodies sleep?
bool doSleep = true;
b2World world(gravity, doSleep);
