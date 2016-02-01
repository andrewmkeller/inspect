#include "stdafx.h"
#include "time.h"
#include "math.h"
#include "Hardware.h"

ImagingResources	CTCSys::IR;

CTCSys::CTCSys()
{
	EventEndProcess = TRUE;
	IR.Acquisition = TRUE;
	IR.UpdateImage = TRUE;
	IR.Inspection = FALSE;
	OPENF("c:\\Projects\\RunTest.txt");
}

CTCSys::~CTCSys()
{
	CLOSEF;
}

void CTCSys::QSStartThread()
{
	EventEndProcess = FALSE;
	//QSProcessEvent = CreateEvent(NULL, FALSE, TRUE, NULL);
	// Image Processing Thread
	QSProcessThreadHandle = CreateThread(NULL, 0L,
		(LPTHREAD_START_ROUTINE)QSProcessThreadFunc,
		this, NULL, (LPDWORD)&QSProcessThreadHandleID);
	ASSERT(QSProcessThreadHandle != NULL);
	SetThreadPriority(QSProcessThreadHandle, THREAD_PRIORITY_HIGHEST);
}

void CTCSys::QSStopThread()
{
	// need to make sure camera acquisiton has stopped
	EventEndProcess = TRUE;
	do {
		Sleep(100);
		// SetEvent(QSProcessEvent);
	} while (EventEndProcess == TRUE);
	CloseHandle(QSProcessThreadHandle);
}

Point2f calcLineIntersection(Size imgSize, float rho, float theta, bool vertical, bool dir)
{
	int dirInt = dir ? 1 : -1;
	float dy1 = -cos(theta)*dirInt;
	float dx1 = sin(theta)*dirInt;
	Point2f point(rho*cos(theta), rho*sin(theta));
	if (vertical)
	{
		float dx2 = (dx1 < 0 ? 0 : imgSize.width) - point.x;
		int dy2 = dy1 * dx2 / dx1;
		return Point(point.x + dx2, point.y + dy2);
	}
	else
	{
		float dy2 = (dy1 < 0 ? 0 : imgSize.height) - point.y;
		int dx2 = dx1 * dy2 / dy1;
		return Point(point.x + dx2, point.y + dy2);
	}
}

Point calcRectIntersection(Size imgSize, float rho, float theta, bool dir)
{
	Point p1 = calcLineIntersection(imgSize, rho, theta, true, dir);
	if (p1.x >= 0 && p1.y >= 0 && p1.x <= imgSize.width && p1.y <= imgSize.height)
		return p1;
	else
		return calcLineIntersection(imgSize, rho, theta, false, dir);
}

void calcCrop(Size& imgSize, vector<Vec2f>& lines, vector<Point2f>& pts, Rect& roi)
{
	Mat edgesC;
	int roiX1 = 0;
	int roiX2 = imgSize.width;
	for (int i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		Point2f p1 = calcRectIntersection(imgSize, rho, theta, true);
		Point2f p2 = calcRectIntersection(imgSize, rho, theta, false);
		if (p1.x < imgSize.width / 2)
		{
			if (p1.x > roiX1)
				roiX1 = p1.x;
		}
		else
		if (p1.x < roiX2)
			roiX2 = p1.x;

		if (p2.x < imgSize.width / 2)
		{
			if (p2.x > roiX1)
				roiX1 = p2.x;
		}
		else
		if (p2.x < roiX2)
			roiX2 = p2.x;
		pts.push_back(p1);
		pts.push_back(p2);
	}
	int border = 10;
	Rect roi2(roiX1 + border, 0, roiX2 - roiX1 - 2*border, imgSize.height);
	roi = roi2;
}

void drawCrop(Mat& img, vector<Point2f> pts, Rect& roi)
{
	for (int i = 0; i < pts.size(); i += 2)
	{
		line(img, pts[i], pts[i + 1], pts[i].x > img.size().width / 2 ? Scalar(0, 0, 255) : Scalar(255, 0, 0), 1, CV_AA);
	}
	rectangle(img, roi, Scalar(0, 255, 0), 1, CV_AA);
}

void calcAndOrDraw(Mat& img, Rect& roi, bool drawResult)
{
	// Crop the image
	Mat gray;
	cvtColor(img, gray, CV_BGR2GRAY); //Grayscale
	Mat detectedEdges;
	blur(gray, detectedEdges, Size(3, 3)); //Blur
	int val2 = 30;
	Canny(detectedEdges, detectedEdges, val2, val2 * 3, 3); //Canny
	vector<Vec2f> lines;
	HoughLines(detectedEdges, lines, 1, CV_PI / 180, 150); //Hough Line

	vector<Point2f> pts;
	calcCrop(img.size(), lines, pts, roi);

	if (drawResult)
		drawCrop(img, pts, roi);
}

void getChannel(Mat& img, Mat& imgChannel, int channel)
{
	Mat channels[3];
	split(img, channels);
	imgChannel = channels[channel];
}

void channelDist(Mat& hsv, Mat& dist, int hueVal, int channel)
{
	Mat channelImg;
	getChannel(hsv, channelImg, channel);
	if (channel == 0)
	{
		Mat dist1, dist2;
		absdiff(channelImg, hueVal, dist1);
		absdiff(channelImg, hueVal + ((hueVal < 90) ? 180 : -180), dist2);
		min(dist1, dist2, dist);
	}
	else
		absdiff(channelImg, hueVal, dist);
}

long QSProcessThreadFunc(CTCSys *QS)
{
	int i;
	int	pass = -1;
	while (QS->EventEndProcess == FALSE) {

#ifdef PTGREY
		if (QS->IR.Acquisition == TRUE) {
			for (i = 0; i < QS->IR.NumCameras; i++) {
				if (QS->IR.pgrCamera[i]->RetrieveBuffer(&QS->IR.PtGBuf[i]) == PGRERROR_OK) {
					QS->QSSysConvertToOpenCV(&QS->IR.AcqBuf[i], QS->IR.PtGBuf[i]);
				}
			}
			for (i = 0; i < QS->IR.NumCameras; i++) {
#ifdef PTG_COLOR
				mixChannels(&QS->IR.AcqBuf[i], 1, &QS->IR.ProcBuf[i], 1, QS->IR.from_to, 3); // Swap B and R channels anc=d copy out the image at the same time.
#else
				QS->IR.AcqBuf[i].copyTo(QS->IR.ProcBuf[i][BufID]);	// Has to copy out of acquisition buffer before processing
#endif
			}
		}
#else
		Sleep(200);
#endif
		// Process Image ProcBuf
		if (QS->IR.Inspection) {
			// Images are acquired into ProcBuf{0] 
			// May need to create child image for processing to exclude background and speed up processing
			
			for (i = 0; i < QS->IR.NumCameras; i++)
			{
				//Obtain image
				Mat imgFull = QS->IR.ProcBuf[i];

				//Crop image
				static Rect roi;
				if (roi.width == 0)
					calcAndOrDraw(imgFull, roi, true);
				Mat imgCropped(imgFull, roi);
				Mat img = imgCropped;

				// Threshold
				Mat imgHsv, d0, d2;
				cvtColor(img, imgHsv, CV_BGR2HSV);
				channelDist(imgHsv, d0, 16, 0);
				bitwise_not(d0, d0);
				getChannel(imgHsv, d2, 2);
				addWeighted(d0, .5, d2, .5, 0, img);
				GaussianBlur(img, img, Size(5, 5), 0);
				static int th = 0;
				if (th == 0)
					th = cv::threshold(img, img, 60, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
				else
					cv::threshold(img, img, th, 255, CV_THRESH_BINARY);

				//Find contours
				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;
				findContours(img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
				img = Mat::zeros(img.size(), CV_8UC3);
				for (int i = 0; i < contours.size(); i++)
					drawContours(img, contours, i, Scalar(255, 255, 255));

				// Find parent contour
				int pretzelContourIndex = -1;
				int minPretzelSize = 1000;
				int largestContourSize = minPretzelSize;
				for (int i = 0; i < hierarchy.size(); i++)
				{
					if (hierarchy[i][3] == -1)
					{
						Moments mm = moments((Mat)contours[i]);
						if (mm.m00 > largestContourSize)
						{
							if (pretzelContourIndex != -1) // if multiple pretzels
							{
								pretzelContourIndex = -2;
								break;
							}
							pretzelContourIndex = i;
							largestContourSize = mm.m00;
							printf("Size: %d\n", (int)mm.m00);
						}
					}
				}
				int pretzelSize = largestContourSize;

				// Evaluate pretzel based on contour children
				int minHoleSize = 10;
				if (pretzelContourIndex != -1 && pretzelContourIndex != -2)
				{
					// Find center of mass
					Moments mm = moments((Mat)contours[pretzelContourIndex]);
					double centerX = (mm.m10 / mm.m00);
					double centerY = (mm.m01 / mm.m00);
					circle(img, Point(centerX, centerY), 4, Scalar(0, 255, 0));

					int borderSize = 100;
					if (centerY > borderSize && centerY < img.size().height - borderSize)
					{
						int numberOfHoles = 0;
						int child = hierarchy[pretzelContourIndex][2];
						while (child != -1)
						{
							if (contours[child].size() > minHoleSize)
								numberOfHoles++;
							child = hierarchy[child][0];
						}
						if (numberOfHoles <= 1)
							pass = 2;
						else if (numberOfHoles == 2)
							pass = 1;
						else if (numberOfHoles == 3)
							pass = 0;
					}
					else
						pass = 3; //no pretzel on belt
				}
				else if (pretzelContourIndex == -1)
					pass = 3; //no pretzel on belt
				else //if (pretzelContourIndex == -2)
					pass = -1; //error

				//Output Image
				if (img.channels() != 3)
					cvtColor(img, img, CV_GRAY2BGR);
				img.copyTo(imgCropped);

				if (imgFull.channels() == 3)
					cvtColor(imgFull, imgFull, CV_BGR2GRAY);
				imgFull.copyTo(QS->IR.OutBuf1[i]);
			}

		}
		// Display Image
		if (QS->IR.UpdateImage) {
			for (i = 0; i<QS->IR.NumCameras; i++) {
				if (!QS->IR.Inspection) {
					// Example of displaying color buffer ProcBuf
					QS->IR.ProcBuf[i].copyTo(QS->IR.DispBuf[i]);
				}
				else {
					// Example of displaying B/W buffer OutBuf1
					QS->IR.OutBuf[0] = QS->IR.OutBuf[1] = QS->IR.OutBuf[2] = QS->IR.OutROI1[i];
					merge(QS->IR.OutBuf, 3, QS->IR.DispROI[i]);
					// Example to show inspection result, print result after the image is copied
					QS->QSSysPrintResult(pass);
				}
			}
			QS->QSSysDisplayImage();
		}
	}
	QS->EventEndProcess = FALSE;
	return 0;
}

void CTCSys::QSSysInit()
{
	long i;
	IR.DigSizeX = 640;
	IR.DigSizeY = 480;
	initBitmapStruct(IR.DigSizeX, IR.DigSizeY);
	// Camera Initialization
#ifdef PTGREY
	IR.cameraConfig.asyncBusSpeed = BUSSPEED_S800;
	IR.cameraConfig.isochBusSpeed = BUSSPEED_S800;
	IR.cameraConfig.grabMode = DROP_FRAMES;			// take the last one, block grabbing, same as flycaptureLockLatest
	IR.cameraConfig.grabTimeout = TIMEOUT_INFINITE;	// wait indefinitely
	IR.cameraConfig.numBuffers = 4;					// really does not matter since DROP_FRAMES is set not to accumulate buffers

	// How many cameras are on the bus?
	if (IR.busMgr.GetNumOfCameras((unsigned int *)&IR.NumCameras) != PGRERROR_OK){	// something didn't work correctly - print error message
		AfxMessageBox(L"Connect Failure", MB_ICONSTOP);
	}
	else {
		IR.NumCameras = (IR.NumCameras > MAX_CAMERA) ? MAX_CAMERA : IR.NumCameras;
		for (i = 0; i < IR.NumCameras; i++) {
			// Get PGRGuid
			if (IR.busMgr.GetCameraFromIndex(0, &IR.prgGuid[i]) != PGRERROR_OK) {
				AfxMessageBox(L"PGRGuID Failure", MB_ICONSTOP);
			}
			IR.pgrCamera[i] = new Camera;
			if (IR.pgrCamera[i]->Connect(&IR.prgGuid[i]) != PGRERROR_OK) {
				AfxMessageBox(L"PConnect Failure", MB_ICONSTOP);
			}
			// Set all camera configuration parameters
			if (IR.pgrCamera[i]->SetConfiguration(&IR.cameraConfig) != PGRERROR_OK) {
				AfxMessageBox(L"Set Configuration Failure", MB_ICONSTOP);
			}
			// Set video mode and frame rate
			if (IR.pgrCamera[i]->SetVideoModeAndFrameRate(VIDEO_FORMAT, CAMERA_FPS) != PGRERROR_OK) {
				AfxMessageBox(L"Video Format Failure", MB_ICONSTOP);
			}
			// Sets the onePush option off, Turns the control on/off on, disables auto control.  These are applied to all properties.
			IR.cameraProperty.onePush = false;
			IR.cameraProperty.autoManualMode = false;
			IR.cameraProperty.absControl = true;
			IR.cameraProperty.onOff = true;
			// Set shutter sppeed
			IR.cameraProperty.type = SHUTTER;
			IR.cameraProperty.absValue = SHUTTER_SPEED;
			if (IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){
				AfxMessageBox(L"Shutter Failure", MB_ICONSTOP);
			}
#ifdef  PTG_COLOR
			// Set white balance (R and B values)
			IR.cameraProperty = WHITE_BALANCE;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.onOff = true;
			IR.cameraProperty.valueA = WHITE_BALANCE_R;
			IR.cameraProperty.valueB = WHITE_BALANCE_B;
			//			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
			//				AfxMessageBox(L"White Balance Failure", MB_ICONSTOP);
			//			}
#endif
			// Set gain values (350 here gives 12.32dB, varies linearly)
			IR.cameraProperty = GAIN;
			IR.cameraProperty.absControl = false;
			IR.cameraProperty.onOff = true;
			IR.cameraProperty.valueA = GAIN_VALUE_A;
			IR.cameraProperty.valueB = GAIN_VALUE_B;
			if (IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){
				AfxMessageBox(L"Gain Failure", MB_ICONSTOP);
			}
			// Set trigger state
			IR.cameraTrigger.mode = 0;
			IR.cameraTrigger.onOff = TRIGGER_ON;
			IR.cameraTrigger.polarity = 0;
			IR.cameraTrigger.source = 0;
			IR.cameraTrigger.parameter = 0;
			if (IR.pgrCamera[i]->SetTriggerMode(&IR.cameraTrigger, false) != PGRERROR_OK){
				AfxMessageBox(L"Trigger Failure", MB_ICONSTOP);
			}
			// Start Capture Individually
			if (IR.pgrCamera[i]->StartCapture() != PGRERROR_OK) {
				char Msg[128];
				sprintf_s(Msg, "Start Capture Camera %d Failure", i);
				AfxMessageBox(CA2W(Msg), MB_ICONSTOP);
			}
		}
		// Start Sync Capture (only need to do it with one camera)
		//		if (IR.pgrCamera[0]->StartSyncCapture(IR.NumCameras, (const Camera**)IR.pgrCamera, NULL, NULL) != PGRERROR_OK) {
		//			AfxMessageBox(L"Start Sync Capture Failure", MB_ICONSTOP);
		//		}
	}
#else
	IR.NumCameras = MAX_CAMERA;
#endif
	Rect R = Rect(0, 0, IR.DigSizeX, IR.DigSizeY);
	// create openCV image
	for (i = 0; i<IR.NumCameras; i++) {
#ifdef PTG_COLOR
		IR.AcqBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		IR.DispBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
		IR.ProcBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC3);
#else 
		IR.AcqBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.DispBuf[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.ProcBuf[i][0].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.ProcBuf[i][1].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
#endif
		IR.AcqPtr[i] = IR.AcqBuf[i].data;
		IR.DispROI[i] = IR.DispBuf[i](R);

		IR.OutBuf1[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.OutROI1[i] = IR.OutBuf1[i](R);
		IR.OutBuf2[i].create(IR.DigSizeY, IR.DigSizeX, CV_8UC1);
		IR.OutROI2[i] = IR.OutBuf2[i](R);
		IR.DispBuf[i] = Scalar(0);
		IR.ProcBuf[i] = Scalar(0);
	}
	IR.from_to[0] = 0;
	IR.from_to[1] = 2;
	IR.from_to[2] = 1;
	IR.from_to[3] = 1;
	IR.from_to[4] = 2;
	IR.from_to[5] = 0;
	QSStartThread();
}

void CTCSys::QSSysFree()
{
	QSStopThread(); // Move to below PTGREY if on Windows Vista
#ifdef PTGREY
	for (int i = 0; i<IR.NumCameras; i++) {
		if (IR.pgrCamera[i]) {
			IR.pgrCamera[i]->StopCapture();
			IR.pgrCamera[i]->Disconnect();
			delete IR.pgrCamera[i];
		}
	}
#endif
}

void CTCSys::initBitmapStruct(long iCols, long iRows)
{
	m_bitmapInfo.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
	m_bitmapInfo.bmiHeader.biPlanes = 1;
	m_bitmapInfo.bmiHeader.biCompression = BI_RGB;
	m_bitmapInfo.bmiHeader.biXPelsPerMeter = 120;
	m_bitmapInfo.bmiHeader.biYPelsPerMeter = 120;
	m_bitmapInfo.bmiHeader.biClrUsed = 0;
	m_bitmapInfo.bmiHeader.biClrImportant = 0;
	m_bitmapInfo.bmiHeader.biWidth = iCols;
	m_bitmapInfo.bmiHeader.biHeight = -iRows;
	m_bitmapInfo.bmiHeader.biBitCount = 24;
	m_bitmapInfo.bmiHeader.biSizeImage =
		m_bitmapInfo.bmiHeader.biWidth * m_bitmapInfo.bmiHeader.biHeight * (m_bitmapInfo.bmiHeader.biBitCount / 8);
}

void CTCSys::QSSysDisplayImage()
{
	SetDIBitsToDevice(ImageDC[0]->GetSafeHdc(), 1, 1,
		m_bitmapInfo.bmiHeader.biWidth,
		::abs(m_bitmapInfo.bmiHeader.biHeight),
		0, 0, 0,
		::abs(m_bitmapInfo.bmiHeader.biHeight),
		IR.DispBuf[0].data,
		&m_bitmapInfo, DIB_RGB_COLORS);
}


#ifdef PTGREY
void CTCSys::QSSysConvertToOpenCV(Mat* openCV_image, Image PGR_image)
{
	openCV_image->data = PGR_image.GetData();	// Pointer to image data
	openCV_image->cols = PGR_image.GetCols();	// Image width in pixels
	openCV_image->rows = PGR_image.GetRows();	// Image height in pixels
	openCV_image->step = PGR_image.GetStride(); // Size of aligned image row in bytes
}
#endif

void CTCSys::QSSysPrintResult(int pass)
{
	if (pass != 3)
	{
		putText(IR.DispBuf[0], (pass == 0) ? "Good" : (pass == 1) ? "Ugly" : (pass == 2) ? "Bad" : "ERROR",
			Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, (pass == 0) ? CV_RGB(0, 255, 0) : (pass == 1) ? CV_RGB(0, 0, 255) : (pass == 2) ? CV_RGB(255, 0, 0) : CV_RGB(128, 128, 128), 2);
	}
}
