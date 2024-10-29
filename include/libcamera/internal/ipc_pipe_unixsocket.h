/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Image Processing Algorithm IPC module using unix socket
 */

#pragma once

#include <map>
#include <memory>

#include "libcamera/internal/ipc_pipe.h"
#include "libcamera/internal/ipc_unixsocket.h"

namespace libcamera {

class Process;
class IPCUnixSocketWrapper;

class IPCPipeUnixSocket : public IPCPipe
{
public:
	IPCPipeUnixSocket(const char *ipaModulePath, const char *ipaProxyWorkerPath);
	~IPCPipeUnixSocket();

	int sendSync(const IPCMessage &in,
		     IPCMessage *out = nullptr) override;

	int sendAsync(const IPCMessage &data) override;

private:
	std::unique_ptr<Process> proc_;
	std::unique_ptr<IPCUnixSocketWrapper> socketWrap_;
};

} /* namespace libcamera */
