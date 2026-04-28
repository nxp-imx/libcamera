/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright 2026 NXP
 *
 * NXP NEO control algorithm interface
 */

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::nxpneo {

/**
 * \enum AlgorithmIrOps
 * \brief Flags for algorithm active stream operations in infrared context
 *
 * \var AlgorithmIrOps::IrOpNone
 * \brief No IR operations
 *
 * \var AlgorithmIrOps::IrOpPrepare
 * \brief Call prepare() for IR context
 *
 * \var AlgorithmIrOps::IrOpProcess
 * \brief Call process() for IR context
 *
 * \var AlgorithmIrOps::IrOpAll
 * \brief Call all operations for IR context
 */

/**
 * \fn Algorithm::irOps
 * \brief Report which operations should be called for IR context
 * \return Bitfield of AlgorithmIrOps flags
 */

/**
 * \fn Algorithm::setIrOps
 * \brief Set which operations should be called for IR context
 * \param[in] ops Bitfield of AlgorithmIrOps flags
 */

} /* namespace ipa::nxpneo */

} /* namespace libcamera */
