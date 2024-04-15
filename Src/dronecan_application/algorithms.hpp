/**
 * This program is free software under the GNU General Public License v3.
 * See <https://www.gnu.org/licenses/> for details.
 * Author: Dmitry Ponomarev <ponomarevda96@gmail.com>
 */

#ifndef SRC_ALGORITHMS_HPP_
#define SRC_ALGORITHMS_HPP_

#include <stdint.h>

static inline void movingAverage(float* prev_avg, float crnt_val, uint16_t size) {
    if (prev_avg != nullptr && size != 0) {
        *prev_avg = (*prev_avg * (size - 1) + crnt_val) / size;
    }
}

#endif  // SRC_ALGORITHMS_HPP_
