/**
 * MIT License
 *
 * Copyright (c) 2023 Thomas Izycki
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */

#ifndef _POSIXMQ_WRAPPER_
#define _POSIXMQ_WRAPPER_

#include <stdio.h>
#include <mqueue.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Create or open a POSIX message queue
 *
 * @param queueName Name of the queue for identification
 * @param queueLength Max amount of items in the queue
 * @param maxMsgLength Size of a message
 * @param errnum errno to indicate the error
 *
 * @return On error, -1 is returned, else the queue descriptor
 */
mqd_t openMqPosix(const char* queueName, const long queueLength,
                  const long maxMsgLength, int* errnum);

/** Connect to an existing POSIX message queue
 *
 * @param queueName Name of the queue for identification
 * @param queueLength Max amount of items in the queue
 * @param maxMsgLength Size of a message
 * @param errnum errno to indicate the error
 *
 * @return On error, -1 is returned, else the queue descriptor
 */
mqd_t connectMqPosix(const char* queueName, const long queueLength,
                     const long maxMsgLength, int* errnum);

/** Create or open a POSIX message queue with O_NONBLOCK set
 *
 * @param queueName Name of the queue for identification
 * @param queueLength Max amount of items in the queue
 * @param maxMsgLength Size of a message
 * @param errnum errno to indicate the error
 *
 * @return On error, -1 is returned, else the queue descriptor
 */
mqd_t openMqPosixNonblock(const char* queueName, const long queueLength,
                          const long maxMsgLength, int* errnum);

/** Remove the oldest message with the highest priority from the message queue
 *
 * @param queueDes Queue descriptor
 * @param msgBuffer Buffer to write the received data
 * @param msgLength Size of to the buffer
 * @param priority Priority associated with the message. If the priority is not NULL,
 * the priority of the selected message is stored in the location referenced by priority
 * @param timeout Maximum amount of time in nanoseconds to wait blocking for data
 * @param errnum errno to indicate the error
 *
 * @return On error, -1 is returned, else the number of bytes read
 */
int rcvMqPosix(int queueDes, char* msgBuffer, size_t msgLength,
               unsigned* priority, int* errnum);

/** Remove the oldest message with the highest priority from the message queue
 *
 * @param queueDes Queue descriptor
 * @param msgBuffer Buffer to write the received data
 * @param msgLength Size of to the buffer
 * @param priority Priority associated with the message. If the priority is not NULL,
 * the priority of the selected message is stored in the location referenced by priority
 * @param errnum errno to indicate the error
 *
 * @return On error or when no message was removed, -1 is returned,
 * else the number of bytes read
 */
int rcvMqPosixTimed(int queueDes, char* msgBuffer, size_t msgLength,
                    unsigned* priority, long timeout, int* errnum);

/** Add a message to the message queue
 *
 * @param queueDes Queue descriptor
 * @param msg Pointer to the message to be added
 * @param msgLength Size of to the message in bytes
 * @param priority Priority associated with the message
 * @param errnum errno to indicate the error
 *
 * @return On error, when the message could not be placed, -1 is returned, else 0
 */
int sendMqPosix(int queueDes, char* msg, size_t msgLength,
                unsigned priority, int* errnum);

/** Add a message to the message queue
 *
 * @param queueDes Queue descriptor
 * @param msg Pointer to the message to be added
 * @param msgLength Size of to the message in bytes
 * @param priority Priority associated with the message
 * @param timeout Time in nanoseconds to wait for a message to placed in the queue
 * @param errnum errno to indicate the error
 *
 * @return On error, when the message could not be placed, -1 is returned, else 0
 */
int sendMqPosixTimed(int queueDes, char* msg, size_t msgLength,
                     unsigned priority, long timeout, int* errnum);

/** Flush the message queue by reading all messages
 *
 * @param queueDes Queue descriptor
 * @param errnum errno to indicate the error
 *
 * @return On error -1 is returned, else 0
 */
int flushMqPosix(int queueDes, int* errnum);

/** Close the message queue descriptor. This does not delete the queue
 *
 * @param queueDes Queue descriptor
 * @param errnum errno to indicate the error
 *
 * @return On error -1 is returned, else 0
 */
int closeMqPosix(int queueDes, int* errnum);

/** Remove the message queue
 *
 * @param queueDes Queue descriptor
 * @param errnum errno to indicate the error
 *
 * @return On error -1 is returned, else 0
 */
int unlinkMqPosix(const char* queueName, int* errnum);

#ifdef __cplusplus
}
#endif

#endif /* _POSIXMQ_WRAPPER_ */
