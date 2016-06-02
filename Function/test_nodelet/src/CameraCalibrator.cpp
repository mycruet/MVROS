#include <test_nodelet/CameraCalibrator.h>
#include <string.h>
#include <stdio.h>
#include <ros/ros.h>

CameraCalibrator::CameraCalibrator(void):flag(0),mustInitUndistort(true)
{
    std::cout<<"'''''''''''''''''''''''''"<<std::endl;
}
CameraCalibrator::CameraCalibrator(cv::Size boardSize):flag(0),mustInitUndistort(true)
{
    std::cout<<"==================="<<std::endl;
    boardSize_ = boardSize;
}


CameraCalibrator::~CameraCalibrator(void)
{
}


void CameraCalibrator::addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners)
{ 

									 // 2D image points from one view 
									 imagePoints.push_back(imageCorners);           
									 // corresponding 3D scene points 
									 objectPoints.push_back(objectCorners); 
} 

bool CameraCalibrator::addChessboardPoints(const cv::Mat &image)
{
	// the points on the chessboard 
	std::vector<cv::Point2f> imageCorners; 
	std::vector<cv::Point3f> objectCorners; 

	// 3D Scene Points: 
	// Initialize the chessboard corners  
	// in the chessboard reference frame 
	// The corners are at 3D location (X,Y,Z)= (i,j,0) 
	for (int i=0; i<boardSize_.height; i++) { 
		for (int j=0; j<boardSize_.width; j++) { 

			objectCorners.push_back(cv::Point3f(i, j, 0.0f)); 
		} 
	} 

	//std::cout<<image.rows<<"X"<<image.cols<<image.type()<<"channel="<<image.channels()<<std::endl;

	// Get the chessboard corners 
	bool found = cv::findChessboardCorners(image, boardSize_, imageCorners); 
        ROS_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>%s---%d---%d", found == true?"true":"false", boardSize_.width, boardSize_.height);
	//std::cout<<"===="<<imageCorners.size()<<"channel="<<image.channels()<<std::endl;

        if(found != true || imageCorners.size() != boardSize_.area())
           return false;

	cv::drawChessboardCorners(image,  		boardSize_, imageCorners,  		found); // corners have been found 
       // cv::imshow("tmp", image);

	cv::Mat edges;

    cvtColor(image, edges, cv::COLOR_BGR2GRAY);

	// Get subpixel accuracy on the corners 

	cv::cornerSubPix(edges, imageCorners,  
		cv::Size(5,5),  
		cv::Size(-1,-1),  
		cv::TermCriteria(cv::TermCriteria::MAX_ITER + 
		cv::TermCriteria::EPS,  
		30,      // max number of iterations  
		0.1));  // min accuracy 

	//检测该图像是否符合标定的标准（即角度、距离等，例如相似度较大的图像不能作为标的参考）
	
		addPoints(imageCorners, objectCorners); 

		pics.push_back(image);

	return found;
} 



int CameraCalibrator::addChessboardPoints(const std::vector<std::string>& filelist) { 

		// the points on the chessboard 
		std::vector<cv::Point2f> imageCorners; 
		std::vector<cv::Point3f> objectCorners; 
		char buffer[256] = {0};
		// 3D Scene Points: 
		// Initialize the chessboard corners  
		// in the chessboard reference frame 
		// The corners are at 3D location (X,Y,Z)= (i,j,0) 
		for (int i=0; i<boardSize_.height; i++) { 
			for (int j=0; j<boardSize_.width; j++) { 

				objectCorners.push_back(cv::Point3f(i, j, 0.0f)); 
			} 
		} 
		

#if 0
		for(int i = 0; i < objectCorners.size(); i++)
		{
			//cv::Point2f *t;
			//t = (cv::Point2f *)imageCorners.pop_back()
			cv::Point3f t = objectCorners[i];
			std::cout<<"("<<t.x<<","<<t.y<<","<<t.z<<")"<<"    ";
			if((i+1)%boardSize.width == 0)
				std::cout<<std::endl;

		}
#endif


		// 2D Image points: 
		cv::Mat image; // to contain chessboard image 
		int successes = 0; 
		// for all viewpoints 
		for (int i=0; i<filelist.size(); i++) { 

			// Open the image 
			std::cout<<filelist[i]<<std::endl;

			image = cv::imread(filelist[i],0); 
			std::cout<<image.rows<<"X"<<image.cols<<image.type()<<std::endl;


			//GaussianBlur(image, image, cv::Size(9, 9), 2, 2);




			// Get the chessboard corners 
			bool found = cv::findChessboardCorners( 
				image, boardSize_, imageCorners); 

			std::cout<<"===="<<imageCorners.size()<<"channel="<<image.channels()<<std::endl;
			
			
			// Get subpixel accuracy on the corners 
#if 1
			cv::cornerSubPix(image, imageCorners,  
				cv::Size(5,5),  
				cv::Size(-1,-1),  
				cv::TermCriteria(cv::TermCriteria::MAX_ITER + 
				cv::TermCriteria::EPS,  
				30,      // max number of iterations  
				0.1));  // min accuracy 
#endif
			

#if 0
			for(int i = 0; i < imageCorners.size(); i++)
			{
				//cv::Point2f *t;
				//t = (cv::Point2f *)imageCorners.pop_back()
				cv::Point2f t = imageCorners[i];
				std::cout<<"("<<t.x<<","<<t.y<<")"<<"    ";
				if((i+1)%boardSize.width == 0)
					std::cout<<std::endl;
			}
#endif


#if 0

			cv::drawChessboardCorners(image,  
				boardSize_, imageCorners,  
				found); // corners have been found 
			imshow(itoa(i,buffer,10), image);
#endif

#if 0

			circle(image,cvPoint(175,151),98,CV_RGB(0,255,255),2,8,0);  
			circle(image,cvPoint(470,440),185,CV_RGB(0,255,255),2,8,0);  
			circle(image,cvPoint(933,630),47,CV_RGB(0,255,255),2,8,0);  
			imshow("circle", image);
			imwrite("circleTest.jpg", image);
#endif

			//If we have a good board, add it to our data 
			if (imageCorners.size() == boardSize_.area()) { 

				// Add image and scene points from one view 
				addPoints(imageCorners, objectCorners); 

				successes++; 
			} 

		} 

		return successes; 
} 


bool CameraCalibrator::calibrate()
{


    mustInitUndistort= true;

    cv::calibrateCamera(objectPoints, // the 3D points
    imagePoints,  // the image points
    boardSize_,    // image size
    A, // output camera matrix
    D,   // output distortion matrix
    rvecs, tvecs, // Rs, Ts
    flag);        // set options

    return true;

}


bool CameraCalibrator::calibrate(cv::Size &imageSize) 
{ 
	// undistorter must be reinitialized 
	mustInitUndistort= true; 

	//Output rotations and translations 
	//std::vector<cv::Mat> rvecs, tvecs; 

	// start calibration 
#if 0
	for(int i = 0; i < imagePoints.size(); i++)
	{
		std::cout<<"=="<<i<<"==pic"<<std::endl;
		for(int j = 0; j < imagePoints[i].size(); j++)
		{
			std::cout<<"("<<imagePoints[i][j].x<<","<<imagePoints[i][j].y<<")"<<"    ";
			//if((j+1)%boardSize.width == 0)
				std::cout<<std::endl;
		}
	}
#endif



		calibrateCamera(objectPoints, // the 3D points 
		imagePoints,  // the image points 
		imageSize,    // image size 
		A, // output camera matrix 
		D,   // output distortion matrix 
		rvecs, tvecs, // Rs, Ts  
		flag);        // set options 
		//检查标定的结果形式,即有几幅图被检测了
		assert(rvecs.size() == imagePoints.size());
		assert(rvecs.size() == tvecs.size());
		std::cout<<"image numbers："<<rvecs.size()<<std::endl;

		std::cout<<"=========A====================="<<std::endl;

		std::cout<<A<<A.type()<<std::endl;
		std::cout<<"=========D====================="<<std::endl;

		std::cout<<D<<D.type()<<std::endl;
#if 0
		for(int i = 0; i < rvecs.size();i++)
		{
			std::cout<<"=========R====================="<<std::endl;
			
			cv::Mat k;
			cv::Rodrigues(rvecs[i],k);
			std::cout<<k<<k.cols<<k.rows<<std::endl;

			std::cout<<"=========T====================="<<std::endl;
			std::cout<<tvecs[i]<<tvecs[i].cols<<tvecs[i].rows<<std::endl;


			cv::Mat HH = mergeByCols(k, tvecs[i]);
			std::cout<<"=========H====================="<<std::endl;
			std::cout<<HH<<std::endl;
			//M.push_back(H);
			cv::Mat MM = A*HH;
			std::cout<<"=========M====================="<<std::endl;
			std::cout<<MM<<std::endl;
			H.push_back(MM);
			//[R    t
			// 0    1]
			cv::Mat kk(k);
			kk.resize(kk.rows + 1,cv::Scalar(0));
			std::cout<<"=========kk====================="<<std::endl;

			std::cout<<kk<<std::endl;

			cv::Mat tt(tvecs[i]);
			tt.resize(tt.rows + 1,cv::Scalar(1));
			std::cout<<"=========tt====================="<<std::endl;

			std::cout<<tt<<std::endl;

			cv::Mat HHH = mergeByCols(kk, tt);
			std::cout<<"=========HHH====================="<<std::endl;
			std::cout<<HHH<<std::endl;
			MF.push_back(HHH);

		}
#endif

// 计算从世界坐标到像素坐标转换的错误率
#if 0

		double totalAvgErr = cv::computeReprojectionErrors(objectPoints, imagePoints,
			rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);


#endif

		

#if 0
		for(auto i:rvecs)
		{

			std::cout<<i<<std::endl;
			cv::Mat k;
			cv::Rodrigues(i,k);
			std::cout<<k<<std::endl;

		}
		
		for(auto i:tvecs)
		{

			std::cout<<i<<std::endl;
			
		}
#endif
		//cv::stereoRectify(A,D,A,D,)


		return true;
} 
cv::Mat CameraCalibrator::remap(const cv::Mat &image) { 

	cv::Mat undistorted; 

	if (mustInitUndistort) { // called once per calibration 
		
		cv::initUndistortRectifyMap( 
			A,  // computed camera matrix 
			D,    // computed distortion matrix 
			cv::Mat(),     // optional rectification (none)  
			cv::Mat(),     // camera matrix to generate undistorted 
			image.size(),  // size of undistorted 
			CV_32FC1,      // type of output map 
			map1, map2);   // the x and y mapping functions 

		mustInitUndistort= false; 
	} 
	// Apply mapping functions 
	cv::remap(image, undistorted, map1, map2,  
		cv::INTER_LINEAR); // interpolation type 


	//imshow("dis", undistorted);
#if 0
	//找到去掉基畸变的角点
	std::vector<cv::Point2f> disimageCorners; 

	// Get the chessboard corners 
	bool found = cv::findChessboardCorners( 
		undistorted,  cv::Size(9, 6), disimageCorners); 

	// Get subpixel accuracy on the corners 
#if 0
	cv::cornerSubPix(undistorted, disimageCorners,  
		cv::Size(5,5),  
		cv::Size(-1,-1),  
		cv::TermCriteria(cv::TermCriteria::MAX_ITER + 
		cv::TermCriteria::EPS,  
		30,      // max number of iterations  
		0.1));  // min accuracy 
#endif

#if 1
	for(int i = 0; i < disimageCorners.size(); i++)
	{
		
		cv::Point2f t = disimageCorners[i];
		std::cout<<"dis points"<<std::endl;
		std::cout<<"("<<t.x<<","<<t.y<<")"<<"    ";
		std::cout<<std::endl;
	}
#endif



#endif
	return undistorted; 
} 


bool CameraCalibrator::checkPrepareForCalibrate()
{
	
    if(pics.size() > 15)
	{
	    return true;
	}
	else
	{
		return false;
	}
}




void  CameraCalibrator::reprojectFromImageToObject()
{
#if 0
	//get first picture first imagePoint [u v]
	cv::Point2f uv = imagePoints[0][0];
	//cv::Point2f uv;
	//uv.x= 175.99;
	//uv.y = 2.98;

	std::cout<<"("<<uv.x<<","<<uv.y<<")"<<"    "<<std::endl;
	//and  objectPoint [X Y Z]
	cv::Point3f XYZ = objectPoints[0][0];
	std::cout<<"("<<XYZ.x<<","<<XYZ.y<<","<<XYZ.z<<")"<<"    "<<std::endl;
	//undis[u' v']
	//cv::Mat p = (cv::Mat_<double>(2,1) << uv.x,uv.y);

	cv::Mat p(uv);
	cv::Mat pp;
	//cv::Mat
	std::cout<<"disM"<<p<<std::endl;
	cv::undistort(p,pp, A, D);
	std::cout<<""<<pp<<std::endl;
#endif
#if 0
	std::vector<cv::Point2f> imgp;
	std::vector<cv::Point3f> wp;
	imgp.push_back(uv);
	wp.push_back(XYZ);	
	cv::Mat camerap;
	cv::Mat imgpp;
	//std::vector<cv::Point2f> imgpp;
	//cv::Mat imgppp;
	//cv::Mat imgpppp;

	
	//cv::projectPoints(cv::Mat(wp),rvecs[0],tvecs[0],A,D,imgppp);
	//cv::projectPoints(cv::Mat(wp),rvecs[0],tvecs[0],A, cv::Mat::zeros(5,1,CV_64FC1),imgpppp);
	//std::cout<<"dis1"<<imgppp<<std::endl;
	//std::cout<<"dis2"<<imgpppp<<std::endl;
	undistortPoints(imgp, camerap, A, D);// ?
	std::cout<<"camerap"<<camerap<<std::endl;
	perspectiveTransform(camerap, imgpp, A);
	//undistortPoints(imgppp, camerap, A, D);
	//perspectiveTransform(camerap, imgpp, A);
	//std::cout<<"dis"<<imgpp<<imgpp.size()<<std::endl;
	//std::cout<<"dis"<<imgpp.size()<<imgpp[0].x<<imgpp[0].y<<std::endl;
	
	//cv::Mat imgppH;
	std::vector<cv::Point3f> imgppH;
    //[u' v' 1]
	cv::convertPointsToHomogeneous(imgpp,imgppH);
	
	cv::Mat imgppHH = cv::Mat(imgppH).reshape(1,3);
	std::cout<<"disH"<<imgppHH<<imgppHH.size()<<std::endl;

#endif
#if 0 
	cv::Mat imgpt3;	
	cv::Mat w2;
	cv::Mat w3;	
	cv::Mat w4;

	cv::projectPoints(cv::Mat(w1),rvecs[0],tvecs[0],A,D,w2);
	cv::projectPoints(cv::Mat(w1),rvecs[0],tvecs[0],A, cv::Mat::zeros(5,1,CV_64FC1),w3);

	//cv::undistortPoints(imgpt1,imgpt3,A,D);
	//cv::Point2f suv = imgpt3[0];
	//std::cout<<"suv"<<imgpt3.size()<<"("<<suv.x<<","<<suv.y<<")"<<"    "<<std::endl;
	//std::cout<<"suv"<<imgpt3<<std::endl;
	std::cout<<"w2"<<w2<<std::endl;
	std::cout<<"w3"<<w3<<std::endl;

	undistortPoints(w2, w4, A, D);
	std::cout<<"w4"<<w4<<std::endl;
	cv::Mat p;
	perspectiveTransform(w4, p, A);
	w4 = p;
	std::cout<<"w4.."<<w4<<std::endl;
#endif
	//因为畸变转换的公式是图像坐标
#if 0
	cv::Mat dism = generateDistorByFivePrama(uv);
	std::cout<<dism<<dism.type()<<dism.size()<<std::endl;
	
    cv::Mat d = (cv::Mat_<double>(6,1) << 1,2,3,4,5,6);
   
	std::cout<<d<<d.size()<<std::endl;
	cv::Mat kk = dism*d;
	std::cout<<kk.type()<<std::endl;
    std::cout<<kk<<std::endl;
#endif
	//Zc 
#if 0
	std::cout<<H.size()<<std::endl;       

	 cv::Mat  rw = H[0].row(2);
	 std::cout<<"..."<<rw<<rw.size()<<rw.rows<<rw.cols<<rw.type()<<std::endl;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	 cv::Mat clt;
	 cv::convertPointsToHomogeneous(wp,clt); 
	 cv::Mat cls;

	 clt.reshape(1,4).convertTo(cls, rw.type());
	
	 std::cout<<"..."<<cls<<cls.size()<<cls.rows<<cls.cols<<cls.type()<<std::endl;       

	 cv::Mat Zc = rw*cls; 
	 std::cout<<"Zc"<<Zc<<std::endl;    
	 double *tz = Zc.ptr<double>(0);
	 std::cout<<"tz"<<tz[0]<<std::endl;    

	 cv::Mat imaguv;
	 imgppHH.convertTo(imaguv, Zc.type());
	 //Zc[u' v' 1]=M

	 imaguv = imaguv*tz[0];
	 std::cout<<"imaguv"<<imaguv<<std::endl;    

	 //reverse A
	 cv::Mat RA;
	 cv::Mat((A.inv())).convertTo(RA, imaguv.type());
	 //Xc,Yc,Zc
	 cv::Mat CamerXYZ = RA*imaguv;
	 std::cout<<"CamerXYZ"<<CamerXYZ<<std::endl;    
	 //[Xc,Yc,Zc,1]
	 CamerXYZ.resize(CamerXYZ.rows + 1, cv::Scalar(1));
	 std::cout<<"CamerXYZ"<<CamerXYZ<<std::endl;  

	 cv::Mat mfr   = MF[0].inv();
	 cv::Mat obXYZ = mfr * CamerXYZ;
	 std::cout<<"obXYZ"<<obXYZ<<std::endl;  
#endif
#if 0
	 std::cout<<H[0].size()<<H[0].rows<<H[0].cols<<H[0].type()<<std::endl;   
	 cv::Mat  rw = H[0].row(0);
	 std::cout<<"..."<<rw<<rw.size()<<rw.rows<<rw.cols<<rw.type()<<std::endl;    
	 double m1[]={0,0,0,0,0};
	 for(int i = 0; i <rw.rows ; i++ )
	 {
		 double *p = rw.ptr<double>(i);
		 for(int j = 0; j < rw.cols; j++)
		 {
			  m1[j+1] = p[j];
		 }
		
	 }
	 std::cout<<m1[1]<<"=="<<m1[2]<<"=="<<m1[4]<<std::endl;  

	 rw = H[0].row(1);
	 std::cout<<"..."<<rw<<rw.size()<<rw.rows<<rw.cols<<rw.type()<<std::endl;    
	 double m2[]={0,0,0,0,0};
	 for(int i = 0; i <rw.rows ; i++ )
	 {
		 double *p = rw.ptr<double>(i);
		 for(int j = 0; j < rw.cols; j++)
		 {
			 m2[j+1] = p[j];
		 }

	 }
	 std::cout<<m2[1]<<"=="<<m2[2]<<"=="<<m2[4]<<std::endl;  

	 rw = H[0].row(2);
	 std::cout<<"..."<<rw<<rw.size()<<rw.rows<<rw.cols<<rw.type()<<std::endl;    
	 double m3[]={0,0,0,0,0};
	 for(int i = 0; i <rw.rows ; i++ )
	 {
		 double *p = rw.ptr<double>(i);
		 for(int j = 0; j < rw.cols; j++)
		 {
			 m3[j+1] = p[j];
		 }

	 }
	 std::cout<<m3[1]<<"=="<<m3[2]<<"=="<<m3[4]<<std::endl;  
	 double a =  m1[1]-m3[1]*uv.x;
	 double b =  m1[2]-m3[2]*uv.x;
	 double c =  m2[1]-m3[1]*uv.y;
	 double d =  m2[2]-m3[2]*uv.y;
	 double e = m3[4]*uv.x - m1[4];
	 double f = m3[4]*uv.y - m2[4];

	  std::cout<<a<<"=="<<b<<"=="<<c<<"=="<<d<<"=="<<e<<"=="<<f<<std::endl;  


	 cv::Mat M = (cv::Mat_<double>(2,2) << a,b,c,d);
	 cv::Mat N = (cv::Mat_<double>(2,1) << e,f);

	 std::cout<<M<<std::endl;  
	 std::cout<<N<<std::endl;  

	 cv::Mat MR = M.inv();
	 cv:: Mat  objXY = MR * N;
	 std::cout<<objXY<<std::endl; 
	 cv:: Mat  objXYT;

	 cv::solve(M,N,objXYT); 
	 std::cout<<objXYT<<std::endl; 

	 cv::Mat rod;
	 cv::Rodrigues(rvecs[0],rod);
	 cv::Mat objtXY = reprojectFromImageToObject2TO3(uv, A, D, rod, tvecs[0]);
	 std::cout<<objtXY<<std::endl; 

#endif
#if 0
	cv::Point3f sump = (0,0,0);
	cv::Point3f p    = (0,0,0);
	int i;

	for(i = 0; i < 54; i++)
	{
		p = getDistanceFromObjToImag(0,i);
		std::cout<<"------------->>>>>>>>>"<<p.x<<"-"<<p.y<<std::endl; 

		sump.x = sump.x + p.x;
		sump.y = sump.y + p.y;
	}
	sump.x = sump.x/i;
	sump.y = sump.y/i;
	std::cout<<sump.x<<"-"<<sump.y<<std::endl; 
#endif
}


void  CameraCalibrator::reprojectFromImageToObject(cv::Mat image)
{
#if 0
	std::vector<cv::Point2f> imageCorners; 
	bool found = cv::findChessboardCorners( 
		image, boardSize_, imageCorners); 

	std::cout<<"===="<<imageCorners.size()<<std::endl;


	// Get subpixel accuracy on the corners 

	cv::cornerSubPix(image, imageCorners,  
		cv::Size(5,5),  
		cv::Size(-1,-1),  
		cv::TermCriteria(cv::TermCriteria::MAX_ITER + 
		cv::TermCriteria::EPS,  
		30,      // max number of iterations  
		0.1));  // min accuracy 


	cv::Point3f sump = (0,0,0);
	cv::Point3f p    = (0,0,0);
	int i;

	for(i = 0; i < imageCorners.size(); i++)
	{
		//p = getDistanceFromObjToImag(0,i);
		std::cout<<"------------->>>>>>>>>"<<p.x<<"-"<<p.y<<std::endl; 

		sump.x = sump.x + p.x;
		sump.y = sump.y + p.y;
	}
	sump.x = sump.x/i;
	sump.y = sump.y/i;
	std::cout<<sump.x<<"-"<<sump.y<<std::endl; 
#endif
}

bool CameraCalibrator::checkCircle(cv::Mat src)
{
	bool detected = false;
	cv::Mat  src_gray;
	cvtColor( src, src_gray, CV_BGR2GRAY );
	GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );
	//GaussianBlur( src, src, cv::Size(9, 9), 2, 2 );
	std::vector<cv::Vec3f> circles;
	HoughCircles( src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows/8, 50, 100, 0, 0);
	//HoughCircles( src, circles, CV_HOUGH_GRADIENT, 1, src.rows/8, 50, 100, 0, 0);
	std::cout<<circles.size()<<std::endl; 


	char cbuffer[100] = {0};
	char rbuffer[100] = {0};




	for( size_t i = 0; i < circles.size(); i++ )
	{
	    detected = true;
		//cv::Mat rod;
		//cv::Rodrigues(rvecs[0],rod);
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		std::cout<<"circle centr"<<"x=="<<circles[i][0]<<"y==="<<circles[i][1]<<std::endl;
		//cv::Point2f p(circles[i][0], circles[i][1]);
	    //cv::Mat objXY = reprojectFromImageToObject2TO3(p, A, D, rod, tvecs[0]);

		//std::cout<<"circle objXY---->"<<objXY<<std::endl; 

		int radius = cvRound(circles[i][2]);
		
		// circle center
		circle( src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
		// circle outline
		circle( src, center, radius, cv::Scalar(0,0,255), 1, 8, 0 );
#if 1
		//draw  center and R world value
		memset(cbuffer, 0, sizeof(cbuffer));
		sprintf(cbuffer, "center=(%d,%d)",center.x, center.y);
		// std::ostringstream str;
		//str.precision(2);
		//using objXY replace of 
		//str << "("<<center.x<<","<<center.y<<")";
	
		//std::cout <<str.str()<<std::endl;
		cv::String centerText(cbuffer);
		//std::cout <<centerText<<std::endl;


		int fontFace = cv::FONT_HERSHEY_PLAIN;
		double fontScale = 1;
		cv::Point org(center.x-5, center.y-10);
		cv::Point rend(center.x + radius, center.y );
		cv::putText(src, centerText,org,fontFace,fontScale, cv::Scalar(0,0,255));

		cv::line(src,center,rend, cv::Scalar(0,0,255));
	
		//str.str("");
		//str <<"r="<<radius;
		
		//std::cout <<str.str()<<std::endl;

		memset(rbuffer, 0, sizeof(cbuffer));
		sprintf(rbuffer, "r=%d", radius);

		cv::String radiusText(rbuffer);
		

		cv::Point rOrg(center.x+radius/2, center.y+15);

		cv::putText(src, radiusText,rOrg,fontFace,fontScale, cv::Scalar(0,0,255));
#endif

	}	//circle(image,cvPoint(175,151),98,CV_RGB(0,255,255),2,8,0);  

	imshow("camera", src);

	return detected;

}










/*
合并列向量维数一样的矩阵

*/
cv::Mat mergeByCols(cv::Mat &a, cv::Mat &b)
{
	assert(a.rows == b.rows && a.type() == b.type());
    //std::cout<<a.type()<<std::endl;
	//std::cout<<a<<std::endl;
	//std::cout<<b<<std::endl;
	cv::Mat M(a.rows, a.cols + b.cols, a.type());
	//cv::Mat k;
	//std::cout<<k<<k.cols<<k.rows<<k.type()<<std::endl;
	int i = 0;

	for(; i < a.cols; i++)
	{
		a.col(i).copyTo(M.col(i));
	}
	for(int j = 0; i < M.cols && j < b.cols;i++,j++)
	{
		b.col(j).copyTo(M.col(i));
	}

	return M;
}

#if 0
cv::Mat generateDistorByFivePrama(cv::Point2f m)
{
    double r2 = m.x *m.x + m.y*m.y;
	double r4 = r2*r2;
	double r6 = r4*r2;
	std::cout<<r2<<"==="<<r4<<"==="<<r6<<std::endl;

	cv::Mat kk = (cv::Mat_<double>(2,6) << 
		m.x*r2,m.x*r4,2*m.x*m.y,r2+2*m.x*m.x,m.x*r6,m.x,
		m.y*r2,m.y*r4,r2+2*m.y*m.y,2*m.x*m.y,m.y*r6,m.y
		);
	return kk;
}
#endif

#if 1
cv::Mat reprojectFromImageToObject2TO3(cv::Point2f imagP, cv::Mat cameraMatrix, cv::Mat disMatrix,
								   cv::Mat RodMatrix, cv::Mat TMatrix, cv::Mat *RandTMaxtix, int flags)
{

	//参数检测以及矩阵内部数据类转换（double)
	//std::cout<<"=========cameraMatrix====================="<<std::endl;
	//std::cout<<cameraMatrix<<std::endl;
	assert(cameraMatrix.rows == 3 && cameraMatrix.cols == 3);
	if(cameraMatrix.type() != CV_64FC1)
	{
		std::cout<<"=========cameraMatrix converto CV_64FC1====================="<<std::endl;
		cameraMatrix.convertTo(cameraMatrix, CV_64FC1);
	}

	//std::cout<<"=========disMatrix====================="<<std::endl;
	//std::cout<<disMatrix<<std::endl;
	assert(disMatrix.rows == 1 && disMatrix.cols == 5);

	//std::cout<<"==========RodMatrixMatrix====================="<<std::endl;
	//std::cout<<RodMatrix<<std::endl;
	assert(RodMatrix.rows == 3 && RodMatrix.cols == 3);
	if(RodMatrix.type()!= CV_64FC1)
	{
		std::cout<<"=========RodMatrix converto CV_64FC1====================="<<std::endl;
		RodMatrix.convertTo(RodMatrix, CV_64FC1);
	}

	

	//std::cout<<"==========TMatrix====================="<<std::endl;
	//std::cout<<TMatrix<<std::endl;
	assert(TMatrix.rows == 3 && TMatrix.cols == 1);
	if(TMatrix.type() != CV_64FC1)
	{
		std::cout<<"=========TMatrix converto CV_64FC1====================="<<std::endl;
		TMatrix.convertTo(TMatrix,CV_64FC1);
	}

	//去掉畸变
	cv::Mat camerap;
	cv::Mat imgpp;
	std::vector<cv::Point2f> imgp;
	//std::cout<<"imgp x==="<<imagP.x<<"y==="<<imagP.y<<std::endl;
	imgp.push_back(imagP);
	cv::undistortPoints(imgp, camerap, cameraMatrix, disMatrix);
	cv::perspectiveTransform(camerap, imgpp, cameraMatrix);

	//转成齐次坐标
	std::vector<cv::Point3f> imgppH;
	cv::convertPointsToHomogeneous(imgpp,imgppH);
	cv::Mat imgppHH = cv::Mat(imgppH).reshape(1,3);//将点的形式转成矩阵形式
	std::cout<<"disH"<<imgppHH<<imgppHH.size()<<std::endl;
	assert(imgppHH.rows == 3 && imgppHH.cols == 1);
	//计算外参数矩阵[R t]
	
	
	cv::Mat externalParam = mergeByCols(RodMatrix, TMatrix);
	//std::cout<<"=========externalParam====================="<<std::endl;
	//std::cout<<externalParam<<std::endl;
	assert(externalParam.rows == 3 && externalParam.cols == 4);
#if 0
	if(RandTMaxtix != NULL)
	{

	}
#endif

   //计算映射矩阵 = 内参*外参数

	cv::Mat HomoP = cameraMatrix * externalParam;

	//std::cout<<"=========HomoP====================="<<std::endl;
	//std::cout<<HomoP<<std::endl;
	assert(HomoP.rows == 3 && HomoP.cols == 4);

/*在映射关系中消去Zc，计算求解两个矩阵

RM * [Xw     =  RN
      Yw]  
	
[m11 -m31u   m12-m32u
 m21-m31v     m22-m32v] =RM 


[m34u - m14
 m34v  - m24] =RN

 */
	double m[4][5];//m[i][j] = mij

	for(int i = 0; i < HomoP.rows; i++)
	{
		double *p = HomoP.ptr<double>(i);
		for(int j = 0; j < HomoP.cols; j++)
		{
			m[i+1][j+1] = p[j];
		}
	}

	cv::Mat RM = (cv::Mat_<double>(2,2) << m[1][1] - m[3][1]*imagP.x, m[1][2] - m[3][2]*imagP.x,
		                                   m[2][1] - m[3][1]*imagP.y, m[2][2] - m[3][2]*imagP.y);
	cv::Mat RN = (cv::Mat_<double>(2,1) << m[3][4]*imagP.x - m[1][4], m[3][4]*imagP.y - m[2][4]);
//求解Xw,Yw
	return (RM.inv())*RN;
								   
}

#endif
//得到第i附图第j个角点的物体坐标与实际物体坐标的差值
cv::Point3f  CameraCalibrator::getDistanceFromObjToImag(int picNum, int cornerNum)
{

	cv::Mat rod;
	cv::Rodrigues(rvecs[picNum],rod);
	cv::Point3f objXYZ = objectPoints[picNum][cornerNum];
	std::cout<<"X---"<<objXYZ.x<<"Y----"<<objXYZ.y<<std::endl;
	//cv::Point3f ret = (0,0,0);
        cv::Point3f ret;
#if 0
	//先整幅图去掉畸变
	cv::Mat dism = undistorteds[picNum];
	assert(dism.channels() == 1);
	std::cout<<"dim"<<std::endl;
	std::cout<<dism<<std::endl;

	int ri = cornerNum/(dism.cols * 2);
	int ci = cornerNum%(dism.cols *2);
	double *pp = dism.ptr<double>(ri);
	cv::Point2f temp2D = (pp[ci], pp[ci+1]);
	//std::cout<<sizeof(double)<<std::endl;//64位下double是8字节
	std::cout<<"x"<<temp2D.x<<"y"<<temp2D.y<<std::endl;
#endif
	
	cv::Mat objXY = reprojectFromImageToObject2TO3(imagePoints[picNum][cornerNum], A, D, rod, tvecs[picNum]);
	std::cout<<objXY<<std::endl; 
	assert(objXY.cols == 1 && objXY.rows == 2);
	double *p = objXY.ptr<double>(0);

	ret.x  = fabs(p[0] - objXYZ.x);

	ret.y  = fabs(p[1] - objXYZ.y);

	//std::cout<<ox<<"-"<<oy<<std::endl; 
	return ret;
}
