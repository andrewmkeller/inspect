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
		this, NULL, (LPDWORD) &QSProcessThreadHandleID);
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
	} while(EventEndProcess == TRUE);
	CloseHandle(QSProcessThreadHandle);
}

long QSProcessThreadFunc(CTCSys *QS)
{
	int		i;
	int	pass = -1;
	while (QS->EventEndProcess == FALSE) {

#ifdef PTGREY
		if (QS->IR.Acquisition == TRUE) {
			for(i=0; i < QS->IR.NumCameras; i++) {
				if (QS->IR.pgrCamera[i]->RetrieveBuffer(&QS->IR.PtGBuf[i]) == PGRERROR_OK) {
					QS->QSSysConvertToOpenCV(&QS->IR.AcqBuf[i], QS->IR.PtGBuf[i]);	
				}
			}
			for(i=0; i < QS->IR.NumCameras; i++) {
#ifdef PTG_COLOR
				mixChannels(&QS->IR.AcqBuf[i], 1, &QS->IR.ProcBuf[i], 1, QS->IR.from_to, 3); // Swap B and R channels anc=d copy out the image at the same time.
#else
				QS->IR.AcqBuf[i].copyTo(QS->IR.ProcBuf[i][BufID]);	// Has to copy out of acquisition buffer before processing
#endif
			}
		}
#else
		Sleep (200);
#endif
		// Process Image ProcBuf
		if (QS->IR.Inspection) {
			// Images are acquired into ProcBuf{0] 
			// May need to create child image for processing to exclude background and speed up processing
			Rect roi(130, 0, 400, 480);
			for(i=0; i < QS->IR.NumCameras; i++) {
				// Example using Canny.  Input is ProcBuf.  Output is OutBuf1
				//Crop

				//
				Mat blurred;
				GaussianBlur(QS->IR.ProcBuf[i], blurred, Size(5, 5), 0);
				Mat hsv;
				cvtColor(blurred, hsv, CV_BGR2HSV);
				vector<Mat> channels;
				split(hsv, channels);
				Mat hue(QS->IR.ProcBuf[i].size(), CV_8U);
				mixChannels(hsv, hue, { 0,0 });
				Mat val(QS->IR.ProcBuf[i].size(), CV_8U);
				mixChannels(hsv, val, { 2,0 });
				Mat valInv;
				bitwise_not(val, valInv);
				Mat sum;
				add(hue, valInv, sum);
				Mat otsu;
				threshold(sum, otsu, 0, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU);
				/*cvtColor(QS->IR.ProcBuf[i], QS->IR.OutBuf1[i], CV_RGB2GRAY);*/
				Mat erodeImg;
				erode(otsu, erodeImg, getStructuringElement(MORPH_CROSS,
					Size(2 * 2 + 1, 2 * 2 + 1),
					Point(2, 2)));
				Mat dialateImg;
				dilate(erodeImg, dialateImg, getStructuringElement(MORPH_CROSS,
					Size(2 * 4 + 1, 2 * 4 + 1),
					Point(4, 4)));
				Mat mask(dialateImg.size(), CV_8U,Scalar(0));
				mask(roi) = 255;

				Mat maskAdded;
				bitwise_and(mask, dialateImg, maskAdded);
				//maskAdded.copyTo(QS->IR.OutBuf1[i]);
				vector<vector<Point> > contours;
				vector<Vec4i> hierarchy;
				findContours(maskAdded, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
				Mat found = Mat::zeros(maskAdded.size(), CV_8U);
				for (int i = 0; i< contours.size(); i++)
				{
					Scalar color = Scalar(255);
					drawContours(found, contours, i, color, 2, 8, hierarchy, 0, Point());
				}
				found.copyTo(QS->IR.OutBuf1[i]);

				//Evaluate based on contours
				bool pretzelFound = false;
				for (int i = 0; i < contours.size(); i++) {
					// find all parent contours
					if (hierarchy[i][3] == -1) {
						if (contours[i].size() < 10)
							continue;
						if (pretzelFound) {
							pass = -1;
							break;
						}
						else {
							pretzelFound = true;
						}
						int numberOfHoles = 0;
						int child = hierarchy[i][2];
						if (child >= 0) {
							while (child != -1) {
								if (contours[child].size() > 10)
									numberOfHoles++;
								child = hierarchy[child][0];
							}
						}
						if (numberOfHoles <= 1) {
							pass = 2;
						}
						else if (numberOfHoles == 2) {
							pass = 1;
						}
						else if (numberOfHoles == 3) {
							pass = 0;
						}
					}
				}
			}

		}
		// Display Image
		if (QS->IR.UpdateImage) {
			for (i=0; i<QS->IR.NumCameras; i++) {
				if (!QS->IR.Inspection) {
					// Example of displaying color buffer ProcBuf
					QS->IR.ProcBuf[i].copyTo(QS->IR.DispBuf[i]);
				} else {
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
	if(IR.busMgr.GetNumOfCameras((unsigned int *)&IR.NumCameras) != PGRERROR_OK){	// something didn't work correctly - print error message
		AfxMessageBox(L"Connect Failure", MB_ICONSTOP);
	} else {
		IR.NumCameras = (IR.NumCameras > MAX_CAMERA) ? MAX_CAMERA : IR.NumCameras;
		for(i = 0; i < IR.NumCameras; i++) {		
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
			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
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
			if(IR.pgrCamera[i]->SetProperty(&IR.cameraProperty, false) != PGRERROR_OK){	
				AfxMessageBox(L"Gain Failure", MB_ICONSTOP);
			}
			// Set trigger state
			IR.cameraTrigger.mode = 0;
			IR.cameraTrigger.onOff = TRIGGER_ON;
			IR.cameraTrigger.polarity = 0;
			IR.cameraTrigger.source = 0;
			IR.cameraTrigger.parameter = 0;
			if(IR.pgrCamera[i]->SetTriggerMode(&IR.cameraTrigger, false) != PGRERROR_OK){	
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
	for(i=0; i<IR.NumCameras; i++) {
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
	for(int i=0; i<IR.NumCameras; i++) {
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
	m_bitmapInfo.bmiHeader.biSize			= sizeof( BITMAPINFOHEADER );
	m_bitmapInfo.bmiHeader.biPlanes			= 1;
	m_bitmapInfo.bmiHeader.biCompression	= BI_RGB;
	m_bitmapInfo.bmiHeader.biXPelsPerMeter	= 120;
	m_bitmapInfo.bmiHeader.biYPelsPerMeter	= 120;
    m_bitmapInfo.bmiHeader.biClrUsed		= 0;
    m_bitmapInfo.bmiHeader.biClrImportant	= 0;
    m_bitmapInfo.bmiHeader.biWidth			= iCols;
    m_bitmapInfo.bmiHeader.biHeight			= -iRows;
    m_bitmapInfo.bmiHeader.biBitCount		= 24;
	m_bitmapInfo.bmiHeader.biSizeImage = 
      m_bitmapInfo.bmiHeader.biWidth * m_bitmapInfo.bmiHeader.biHeight * (m_bitmapInfo.bmiHeader.biBitCount / 8 );
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
	putText(IR.DispBuf[0], (pass == 0) ? "Good" : (pass == 1)? "Ugly" : (pass== 2) ? "Bad" : "ERROR", 
		Point(10, 30), FONT_HERSHEY_SIMPLEX, 1, (pass == 0) ? CV_RGB(0, 255, 0) : (pass == 1) ? CV_RGB(0, 0, 255) : (pass == 2) ? CV_RGB(255, 0, 0) : CV_RGB(128, 128, 128), 2);
}
