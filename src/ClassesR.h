b2Fixture *_paddleFixture;

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
	paddle(double x);
	~paddle() {};
	b2Body* body;
	int tag;
};
//Required for Round 3
class movingblock
{
	public:
	movingblock();
	setVelocity(double velx , double vely);
	~movingblock() {};
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

paddle::paddle(double x)
{
	tag=2;
	b2BodyDef bodyDef;
	bodyDef.type = b2_staticBody;
	bodyDef.position.Set(x,WORLDH-2);
	bodyDef.userData=(void *)tag;
	body = world.CreateBody(&bodyDef);
	/* Rectangular */
	b2PolygonShape paddleShape;
	paddleShape.SetAsBox(WORLDW/12, THICKNESS*2);
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
	bodyDef.type = b2_kinematicBody;
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
	body->SetLinearVelocity(b2Vec2(0.5,0));
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