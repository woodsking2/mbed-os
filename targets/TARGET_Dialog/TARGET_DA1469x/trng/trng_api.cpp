#include "trng_api.h"
#include <array>
#include <cstring>
#include "gsl/gsl"
#include "mbed_debug.h"
#include "Mutex.h"
#include "semaphore.h"
extern "C"
{
#include "default_config.h"
#include "hw_trng.h"
}
using namespace std;
using namespace rtos;
using namespace gsl;
namespace
{
Mutex mutex;
Semaphore sem;
void trng_fifo_full()
{
    sem.release();
}
} // namespace
/** Initialize the TRNG peripheral
 *
 * @param obj The TRNG object
 */
void trng_init(trng_t *obj)
{
}

/** Deinitialize the TRNG peripheral
 *
 * @param obj The TRNG object
 */
void trng_free(trng_t *obj)
{
}

/** Get random data from TRNG peripheral
 *
 * @param obj The TRNG object
 * @param output The pointer to an output array
 * @param length The size of output data, to avoid buffer overwrite
 * @param output_length The length of generated data
 * @return 0 success, -1 fail
 */
int trng_get_bytes(trng_t *obj, uint8_t *output, size_t length, size_t *output_length)
{
    mutex.lock();
    auto _ = finally([&]() {
        hw_trng_disable();
        mutex.unlock();
    });
    int fifo_count{};
    int fifo_byte{};
    size_t generated_count{};
    array<uint32_t, 32> fifo;
    do
    {
        // debug("wait\n");
        hw_trng_disable();
        hw_trng_enable(trng_fifo_full);
        sem.acquire();
        // debug("got\n");
        hw_trng_get_numbers(fifo.data(), fifo.size() * 4);
        fifo_count = fifo.size(); // hw_trng_get_fifo_level();
        Ensures(fifo_count <= fifo.size());
        fifo_byte = fifo_count * 4;
        if (length - generated_count < fifo_byte)
        {
            fifo_byte = length - generated_count;
        }
        // for (auto value : fifo)
        // {
        //     debug("%d ", value);
        // }
        // debug("\n");
        debug("generated_count %d fifo_byte:%d %d\n", generated_count, fifo_byte, *fifo.data());
        memcpy(&output[generated_count], fifo.data(), fifo_byte);
        generated_count += fifo_byte;
    } while (generated_count < length);
    if (output_length != 0)
    {
        *output_length = generated_count;
    }

    return 0;
}
