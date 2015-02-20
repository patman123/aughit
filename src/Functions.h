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
	return bin; //Result is returned
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

bool IsValidPoint(int i, int j, int r, int c)
{
	if(i<0||i>=r||j<0||j>=c)
		return false;
	else
		return true;
}

static double findangle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	double dx1 = pt1.x - pt0.x;
	double dy1 = pt1.y - pt0.y;
	double dx2 = pt2.x - pt0.x;
	double dy2 = pt2.y - pt0.y;
	return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void GetBlobs(Mat img, vector<blob>& blobs){
	int i,j,k,l,r = img.rows,c = img.cols,id=1;
	vector<vector<int> > pixel_ID(r,vector<int>(c,-1)); //Stores ID of a pixel; -1 means unvisited
	queue<pt> open_list; //Breadth-First-Search hence queue of points

	for(i=1;i<r-1;i++){
		for(j=1;j<c-1;j++){
			if(img.at<uchar>(i,j)==0||pixel_ID[i][j]>-1)
				continue;
			pt start = {j,i};

			open_list.push(start);
			int sum_x=0,sum_y=0,n_pixels=0,max_x=0,max_y=0;
			int min_x = c+1, min_y=r+1;
			while(!open_list.empty()){
				//Dequeue the element at the head of the queue
				pt top = open_list.front();
				open_list.pop();
				pixel_ID[top.y][top.x] = id;
				n_pixels++;
				//To obtain the bounding box of the blob w.r.t the original image
				min_x = (top.x<min_x)?top.x:min_x;
				min_y = (top.y<min_y)?top.y:min_y;
				max_x = (top.x>max_x)?top.x:max_x;
				max_y = (top.y>max_y)?top.y:max_y;
				// sum_y+=top.y; sum_x+=top.x;

				//Add the 8-connected neighbours that are yet to be visited, to the queue
				for(k=top.y-1;k<=top.y+1;k++){
					for(l=top.x-1;l<=top.x+1;l++){
						if(IsValidPoint(k,l,r,c)==false)
							continue;
						if(img.at<uchar>(k,l)==0||pixel_ID[k][l]>-1)
							continue;
						pt next = {l,k};
						pixel_ID[k][l] = id;
						open_list.push(next);
					}
				}
			}
			if(n_pixels < 100) //At least 100 pixels
				continue;
			blob nextcentre = {min_x,max_x,min_y,max_y,(min_x+max_x)/2,(min_y+max_y)/2/*sum_x/n_pixels*//*,sum_y/n_pixels*/,id};
			blobs.push_back(nextcentre);
			id++;
		}
}

	// cout<<blobs.size(); //To test correctness; can use the vector as desired
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