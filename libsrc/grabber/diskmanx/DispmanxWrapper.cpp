// QT includes
#include <QDebug>
#include <QDateTime>

// Hyperion includes
#include <hyperion/Hyperion.h>
#include <hyperion/ImageProcessorFactory.h>
#include <hyperion/ImageProcessor.h>

// Dispmanx grabber includes
#include <grabber/DispmanxWrapper.h>
#include <grabber/DispmanxFrameGrabber.h>

// STL includes
#include <iostream>

DispmanxWrapper::DispmanxWrapper(const unsigned grabWidth, const unsigned grabHeight, const unsigned updateRate_Hz, const int priority, Hyperion * hyperion) :
	_updateInterval_ms(1000/updateRate_Hz),
	_timeout_ms(2 * _updateInterval_ms),
	_priority(priority),
	_timer(),
	_image(grabWidth, grabHeight),
	_frameGrabber(new DispmanxFrameGrabber(grabWidth, grabHeight)),
	_processor(ImageProcessorFactory::getInstance().newImageProcessor()),
	_ledColors(hyperion->getLedCount(), ColorRgb{0,0,0}),
	_hyperion(hyperion)
{
	// Configure the timer to generate events every n milliseconds
	_timer.setInterval(_updateInterval_ms);
	_timer.setSingleShot(false);

	_processor->setSize(grabWidth, grabHeight);
	_forward = _hyperion->getForwarder()->protoForwardingEnabled();

	// Connect the QTimer to this
	QObject::connect(&_timer, SIGNAL(timeout()), this, SLOT(action()));

	//Activates the tty connection with the Arduino (ref: https://playground.arduino.cc/Interfacing/CPlusPlus)
	std::system("stty -F /dev/ttyACM0 cs8 460800 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke noflsh -ixon -crtscts");
	//Opens the tty connection as an ofstream
	arduino.open("/dev/ttyACM0");
	// set header bytes
	serialHeader[0] = '\xde';
	serialHeader[1] = '\xad';
	serialHeader[2] = '\xbe';
	serialHeader[3] = '\xef';
}

DispmanxWrapper::~DispmanxWrapper()
{
	// Cleanup used resources (ImageProcessor and FrameGrabber)
	delete _processor;
	delete _frameGrabber;

	// close serial connection
	arduino.close();
}

void DispmanxWrapper::start()
{
	// Start the timer with the pre configured interval
	_timer.start();
}

void DispmanxWrapper::action()
{
	// Grab frame into the allocated image
	_frameGrabber->grabFrame(_image);

	if ( _forward )
	{
		Image<ColorRgb> image_rgb;
		_image.toRgb(image_rgb);
		emit emitImage(_priority, image_rgb, _timeout_ms);
	}
    
	// get colour
	for(int i=0; i<512; i++){
		int coordX = 32; int coordY = 18; int lengthX = 64;
		int colourIndex = coordY*lengthX + coordX;
		const auto& pixel = _image.memptr()[colourIndex];
		const uint8_t pixR = uint8_t(pixel.red);
		const uint8_t pixG = uint8_t(pixel.green);
		const uint8_t pixB = uint8_t(pixel.blue);
	}
	int coordX = 32; int coordY = 18; int lengthX = 64;
	int colourIndex = coordY*lengthX + coordX;
	const auto& pixel = _image.memptr()[colourIndex];
	const uint8_t pixR = uint8_t(pixel.red);
	const uint8_t pixG = uint8_t(pixel.green);
	const uint8_t pixB = uint8_t(pixel.blue);

	// arduino send led data test
	for(int i=0; i<4; i++){
		arduino << serialHeader[i]; // send header
	}
	for(int i=0; i<32; i++){
		arduino << pixR; // channel 1
		arduino << pixG; // 2
		arduino << pixB; // 3
	}
	std::cout << "DISKMANXGRABBER TEST DONE: rgb(" << pixR <<","<< pixG <<","<< pixB <<")"<< std::endl;
	
	//_processor->process(_image, _ledColors);
	//_hyperion->setColors(_priority, _ledColors, _timeout_ms);
}

void DispmanxWrapper::stop()
{
	// Stop the timer, effectivly stopping the process
	_timer.stop();
}

void DispmanxWrapper::setGrabbingMode(const GrabbingMode mode)
{
	switch (mode)
	{
	case GRABBINGMODE_VIDEO:
	case GRABBINGMODE_PAUSE:
		_frameGrabber->setFlags(DISPMANX_SNAPSHOT_NO_RGB|DISPMANX_SNAPSHOT_FILL);
		start();
		break;
	case GRABBINGMODE_AUDIO:
	case GRABBINGMODE_PHOTO:
	case GRABBINGMODE_MENU:
	case GRABBINGMODE_INVALID:
		_frameGrabber->setFlags(0);
		start();
		break;
	case GRABBINGMODE_OFF:
		stop();
		break;
	}
}

void DispmanxWrapper::setVideoMode(const VideoMode mode)
{
	_frameGrabber->setVideoMode(mode);
}

void DispmanxWrapper::setCropping(const unsigned cropLeft, const unsigned cropRight,
	const unsigned cropTop, const unsigned cropBottom)
{
	_frameGrabber->setCropping(cropLeft, cropRight, cropTop, cropBottom);
}
