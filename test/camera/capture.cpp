/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * libcamera Camera API tests
 */

#include <iostream>

#include <libcamera/framebuffer_allocator.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "camera_test.h"
#include "test.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

static int dumpCount = 0;
static int fileSize = 0;
static void DumpData(void *buf, uint32_t bufSize)
{
	int  fd = -1;
	const char *fileName = "/data/dump.yuyv";

	dumpCount++;
	if (dumpCount > 10)
		return;


	if ((buf == NULL) || (bufSize == 0))
		return;

	fd = open(fileName, O_CREAT|O_APPEND|O_WRONLY, S_IRWXU|S_IRWXG);
	//fd = open(fileName, O_CREAT|O_WRONLY, S_IRWXU|S_IRWXG);
	if (fd < 0) {
		cout << "DumpData, can't open file " << fileName << endl;
		 return;
	}

	int ret = write(fd, buf, bufSize);
	if (ret < 0)
		cout << "DumpData, write failed, ret " << ret << endl;

	close(fd);

	fileSize += bufSize;
	cout << "==== dumpCount " << dumpCount << ", bufSize " << bufSize << ", fileSize " << fileSize << endl;

	return;
}

#define OV560_8MN_NAME "/base/soc@0/bus@30800000/i2c@30a40000/ov5640_mipi@3c"
#define OV560_8QM_NAME "/base/bus@58000000/i2c@58226000/ov5640_mipi@3c"
#define OV560_95_NAME "/base/soc@0/bus@42000000/i2c@42530000/ov5640_mipi@3c"
#define AP1302_95_NAME "/base/soc/bus@42000000/i2c@42530000/ap1302_mipi@3c"

#define CAP_FMT PixelFormat::fromString("NV12")
#define CAP_WIDTH 1280
#define CAP_HEIGHT 720
#define CAP_FRAME_SIZE (CAP_WIDTH * CAP_HEIGHT * 3 / 2)

namespace {

class Capture : public CameraTest, public Test
{
public:
	Capture()
		: CameraTest(AP1302_95_NAME)
	{
	}

protected:
	unsigned int completeBuffersCount_;
	unsigned int completeRequestsCount_;

	void dumpBuffer(FrameBuffer *buffer)
	{
		auto plans = buffer->planes();
		FrameBuffer::Plane plan = plans[0];
		void *virtAddr = (void*)mmap(NULL, plan.length, PROT_READ | PROT_WRITE, MAP_SHARED, plan.fd.get(), plan.offset);
		cout << "plan offset " << plan.offset << " length " << plan.length << " fd " << plan.fd.get() << " virt addr " << virtAddr << endl;

		DumpData(virtAddr, plan.length);
		munmap(virtAddr, plan.length);

		if (plans.size() > 1) {
			FrameBuffer::Plane plan2 = plans[1];
			void *virtAddr2 = (void*)mmap(NULL, plan2.length, PROT_READ | PROT_WRITE, MAP_SHARED, plan2.fd.get(), plan2.offset);
			cout << "plan2 offset " << plan2.offset << " length " << plan2.length << " fd " << plan2.fd.get() << " virt addr " << virtAddr2 << endl;

			DumpData(virtAddr2, plan2.length);
			munmap(virtAddr2, plan2.length);
		}
	}

	void bufferComplete([[maybe_unused]] Request *request,
			    FrameBuffer *buffer)
	{
		if (buffer->metadata().status != FrameMetadata::FrameSuccess)
			return;

		completeBuffersCount_++;
		dumpBuffer(buffer);
	}

	void requestComplete(Request *request)
	{
		if (request->status() != Request::RequestComplete)
			return;

		const Request::BufferMap &buffers = request->buffers();

		completeRequestsCount_++;

		/* Create a new request. */
		const Stream *stream = buffers.begin()->first;
		FrameBuffer *buffer = buffers.begin()->second;

		request->reuse();
		request->addBuffer(stream, buffer);
		camera_->queueRequest(request);
	}


	int init() override
	{
		if (status_ != TestPass)
			return status_;

		config_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!config_ || config_->size() != 1) {
			cout << "Failed to generate default configuration" << endl;
			return TestFail;
		}

		allocator_ = new FrameBufferAllocator(camera_);

		return TestPass;
	}

	void cleanup() override
	{
		delete allocator_;
	}

	int run() override
	{
		StreamConfiguration &cfg = config_->at(0);

#ifdef ANDROID
		cfg.pixelFormat = CAP_FMT; 
		cfg.size.width = CAP_WIDTH;
		cfg.size.height = CAP_HEIGHT;
#endif

		if (camera_->acquire()) {
			cout << "Failed to acquire the camera" << endl;
			return TestFail;
		}

		if (camera_->configure(config_.get())) {
			cout << "Failed to set default configuration" << endl;
			return TestFail;
		}

		Stream *stream = cfg.stream();

		int ret = allocator_->allocate(stream);
		if (ret < 0)
			return TestFail;

		for (const std::unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
			std::unique_ptr<Request> request = camera_->createRequest();
			if (!request) {
				cout << "Failed to create request" << endl;
				return TestFail;
			}

			if (request->addBuffer(stream, buffer.get())) {
				cout << "Failed to associate buffer with request" << endl;
				return TestFail;
			}

			requests_.push_back(std::move(request));
		}

		completeRequestsCount_ = 0;
		completeBuffersCount_ = 0;

		camera_->bufferCompleted.connect(this, &Capture::bufferComplete);
		camera_->requestCompleted.connect(this, &Capture::requestComplete);

		if (camera_->start()) {
			cout << "Failed to start camera" << endl;
			return TestFail;
		}

		for (std::unique_ptr<Request> &request : requests_) {
			if (camera_->queueRequest(request.get())) {
				cout << "Failed to queue request" << endl;
				return TestFail;
			}
		}

		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();

		Timer timer;
		timer.start(20000ms);
    cout << "==== start timer" << endl;
		while (timer.isRunning())
			dispatcher->processEvents();
    cout << "==== stop timer" << endl;

		unsigned int nbuffers = allocator_->buffers(stream).size();

		if (completeRequestsCount_ < nbuffers * 2) {
			cout << "Failed to capture enough frames (got "
			     << completeRequestsCount_ << " expected at least "
			     << nbuffers * 2 << ")" << endl;
			return TestFail;
		}

		if (completeRequestsCount_ != completeBuffersCount_) {
			cout << "Number of completed buffers and requests differ" << endl;
			return TestFail;
		}

		if (camera_->stop()) {
			cout << "Failed to stop camera" << endl;
			return TestFail;
		}

		return TestPass;
	}

	std::vector<std::unique_ptr<Request>> requests_;

	std::unique_ptr<CameraConfiguration> config_;
	FrameBufferAllocator *allocator_;
};

} /* namespace */

TEST_REGISTER(Capture)
