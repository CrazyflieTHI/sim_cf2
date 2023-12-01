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

#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include "posixmq_wrapper.h"
#include <errno.h>

/* Read write permissions */
#define QUEUE_PERMISSIONS 0660


mqd_t openMqPosix(const char* queueName, const long queueLength,
                  const long maxMsgLength, int* errnum) {
    mqd_t posixQueue;
    struct mq_attr attr;

    /* Set the attributes for the queue */
    attr.mq_flags = 0;
    attr.mq_maxmsg = queueLength;
    attr.mq_msgsize = maxMsgLength;
    attr.mq_curmsgs = 0;

    posixQueue = mq_open(queueName, O_RDWR | O_CREAT, QUEUE_PERMISSIONS, &attr);

    *errnum = errno;

    return posixQueue;
}

mqd_t connectMqPosix(const char* queueName, const long queueLength,
                     const long maxMsgLength, int* errnum) {
    mqd_t posixQueue;
    struct mq_attr attr;

    /* Set the attributes for the queue */
    attr.mq_flags = 0;
    attr.mq_maxmsg = queueLength;
    attr.mq_msgsize = maxMsgLength;
    attr.mq_curmsgs = 0;

    posixQueue = mq_open(queueName, O_RDONLY, QUEUE_PERMISSIONS, &attr);

    *errnum = errno;

    return posixQueue;
}

mqd_t openMqPosixNonblock(const char* queueName, const long queueLength,
                          const long maxMsgLength, int* errnum) {
    mqd_t posixQueue;
    struct mq_attr attr;

    /* Set the attributes for the queue */
    attr.mq_flags = 0;
    attr.mq_maxmsg = queueLength;
    attr.mq_msgsize = maxMsgLength;
    attr.mq_curmsgs = 0;

    posixQueue = mq_open(queueName, O_RDWR | O_CREAT | O_NONBLOCK,
                         QUEUE_PERMISSIONS, &attr);

    *errnum = errno;

    return posixQueue;
}

int rcvMqPosix(int queueDes, char* msgBuffer, size_t msgLength,
               unsigned* priority, int* errnum) {
    mqd_t posixQueueDes = (mqd_t)queueDes;

    int ret = mq_receive(posixQueueDes, msgBuffer, msgLength, priority);

    *errnum = errno;

    return ret;
}

int rcvMqPosixTimed(int queueDes, char* msgBuffer, size_t msgLength,
                    unsigned* priority, long timeout, int* errnum) {
    mqd_t posixQueueDes = (mqd_t)queueDes;
    struct timespec ts;
    ts.tv_sec = time(NULL);
    ts.tv_nsec = timeout;

    int ret = mq_timedreceive(posixQueueDes, msgBuffer, msgLength, priority, &ts);

    *errnum = errno;

    return ret;
}

int sendMqPosix(int queueDes, char* msg, size_t msgLength,
                unsigned priority, int* errnum) {
    mqd_t posixQueueDes = (mqd_t)queueDes;

    int ret = mq_send(posixQueueDes, msg, msgLength, priority);

    *errnum = errno;

    return ret;
}

int sendMqPosixTimed(int queueDes, char* msg, size_t msgLength,
                     unsigned priority, long timeout, int* errnum) {
    mqd_t posixQueueDes = (mqd_t)queueDes;
    struct timespec ts;
    ts.tv_sec = time(NULL);
    ts.tv_nsec = timeout;

    int ret = mq_timedsend(posixQueueDes, msg, msgLength, priority, &ts);

    *errnum = errno;

    return ret;
}

int flushMqPosix(int queueDes, int* errnum) {
    struct mq_attr attr;

    /* Check if the queue exists */
    if (mq_getattr(queueDes, &attr) == -1) {
        // Queue does not exist or other error
        *errnum = errno;
        if (*errnum == ENOENT) {
            // Queue does not exist, treat it as a success for this function
            return 0;
        } else {
            // Other error, return failure
            return -1;
        }
    }

    // The queue exists, flush it
    while (attr.mq_curmsgs > 0) {
        // The queue is not empty, receive a message
        char buffer[attr.mq_msgsize];
        if (mq_receive(queueDes, buffer, attr.mq_msgsize, NULL) == -1) {
            // Error receiving message
            *errnum = errno;
            return -1;
        }

        // Update queue attributes
        if (mq_getattr(queueDes, &attr) == -1) {
            // Error getting attributes
            *errnum = errno;
            return -1;
        }
    }

    // Queue is now empty
    return 0;
}

int closeMqPosix(int queueDes, int* errnum) {
    struct mq_attr attr;
    int ret = 0;

    /* Check if the queue exists */
    if(mq_getattr(queueDes, &attr) == 0)
    {
        ret = mq_close(queueDes);
    }

    *errnum = errno;

    return ret;
}

int unlinkMqPosix(const char* queueName, int* errnum) {
    int ret = mq_unlink(queueName);

    *errnum = errno;

    return ret;
}
