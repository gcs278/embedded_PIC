#include "i2c_queue.h"

unsigned char createQueue(i2c_queue* queue, unsigned char size) {
    queue->size = size;
    queue->front = 0;
    queue->end = -1;
    queue->elements[queue->size];
}

unsigned char putQueue(i2c_queue* queue, unsigned char element) {

    // Make sure it isn't full
    if ( queue->end == queue->size - 1) {
        return 0;
    } else {
        // Either first thing or not
        if (queue->end == -1) {
            queue->elements[0] = element;
            queue->end++; // Increment end
        } else {
            queue->end++;
            queue->elements[queue->end] = element;

        }
        return 1;
    }
}

unsigned char getQueue(i2c_queue* queue, unsigned char* element) {
    // Check if we have something
    if ( queue->end == -1 ) {
        return 0;
    } else {
        // Point element to first one
        *element = queue->elements[0];
        // Move the queue
        int i;
        for (i = 0; i < queue->end; i++)
            queue->elements[i] = queue->elements[i+1];
        queue->end--;
        
        return 1;
    }
}

unsigned char isEmpty(i2c_queue* queue) {
    return (queue->end == -1);
}