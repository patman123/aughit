#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
                        
Mat     img;
int     is_data_ready = 0;
int     listenSock, connectSock;
int 	listenPort;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void* streamServer(void* arg);
void  quit(string msg, int retval);
   
int main(int argc, char** argv)
{
        pthread_t thread_s;
        int width, height, key;
	width = 640;  
	height = 480; 
 	
	if (argc != 2) {
                quit("UWrong input", 0);
        }
	
	listenPort = atoi(argv[1]);
 	
        img = Mat::zeros( height,width, CV_8UC1);
        
        if (pthread_create(&thread_s, NULL, streamServer, NULL)) {
                quit("pthread_create failed.", 1);
        }

        cout << "\n-->Press 'q' to quit." << endl;
        namedWindow("stream_server", CV_WINDOW_AUTOSIZE);

        while(key != 'q') {

                pthread_mutex_lock(&mutex);
                        if (is_data_ready) {
                                imshow("stream_server", img);
                                is_data_ready = 0;
                        }
                pthread_mutex_unlock(&mutex);
                key = waitKey(10);
        }

        if (pthread_cancel(thread_s)) {
                quit("pthread_cancel failed.", 1);
        }

        destroyWindow("stream_server");
        quit("NULL", 0);
}
void* streamServer(void* arg)
{
        struct  sockaddr_in   serverAddr,  clientAddr;
        socklen_t             clientAddrLen = sizeof(clientAddr);

        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

        if ((listenSock = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
            quit("socket() failed.", 1);
        }
            
        serverAddr.sin_family = PF_INET;
        serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
        serverAddr.sin_port = htons(listenPort);

        if (bind(listenSock, (sockaddr*)&serverAddr, sizeof(serverAddr)) == -1) {
                quit("bind() failed", 1);
        }
                
        if (listen(listenSock, 5) == -1) {
                quit("listen() failed.", 1);
        }
        
        int  imgSize = img.total()*img.elemSize();
        char sockData[imgSize];
        int  bytes=0;
                
        while(1)
        {
	        cout << "-->Waiting for TCP connection on port " << listenPort << " ...\n\n";
	  	
	        if ((connectSock = accept(listenSock, (sockaddr*)&clientAddr, &clientAddrLen)) == -1) {
	                quit("accept() failed", 1);
	        }else{
		    	cout << "-->Receiving image from " << inet_ntoa(clientAddr.sin_addr) << ":" << ntohs(clientAddr.sin_port) << "..." << endl;
		}
		
		memset(sockData, 0x0, sizeof(sockData));
		
		while(1){

                	for (int i = 0; i < imgSize; i += bytes) {
                        	if ((bytes = recv(connectSock, sockData +i, imgSize  - i, 0)) == -1) {
 	                              	quit("recv failed", 1);
				}
                	}
                	pthread_mutex_lock(&mutex);
                        	for (int i = 0;  i < img.rows; i++) {
                        	        for (int j = 0; j < img.cols; j++) {
                        	                (img.row(i)).col(j) = (uchar)sockData[((img.cols)*i)+j];
                        	        }
                        	}
                        	is_data_ready = 1;
				memset(sockData, 0x0, sizeof(sockData));
                	pthread_mutex_unlock(&mutex);
		}
	}

        pthread_testcancel();
	
        usleep(1000);
	
}
void quit(string msg, int retval)
{
        if (retval == 0) {
                cout << (msg == "NULL" ? "" : msg) << "\n" <<endl;
        } else {
                cerr << (msg == "NULL" ? "" : msg) << "\n" <<endl;
        }
         
        if (listenSock){
                close(listenSock);
        }

        if (connectSock){
                close(connectSock);
        }
                                
        if (!img.empty()){
                (img.release());
        }
                
        pthread_mutex_destroy(&mutex);
        exit(retval);
}

