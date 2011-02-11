/*
 * testCams.cpp
 *
 *  Created on: 14 Oct 2010
 *      Author: tmc43
 */

#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <getopt.h>
#include <unistd.h>
#include <ellLib.h>


#include <dc1394/dc1394.h>
/** Number of image buffers the dc1394 library will use internally */
#define FDC_DC1394_NUM_BUFFERS 5
/** Print an errorcode to stderr. */
#define ERR(errCode) if (errCode != 0) fprintf(stderr, "ERROR [%s:%d]: dc1394 code: %d\n", __FILE__, __LINE__, errCode)
#define DEBUG(errCode) printf("%3d: " #errCode "\n", __LINE__); ERR(errCode)

typedef struct camNode_t {
    ELLNODE node;
    uint32_t generation;
    uint64_t *newguid;
    dc1394camera_t *cam;
}camNode_t;

/** dc1394 handle to the firewire bus.  */
static dc1394_t * dc1394fwbus = NULL;
/** List of dc1394 camera handles. */
static dc1394camera_list_t * dc1394camList = NULL;

void reset_bus()
{
    dc1394_t * d;
    dc1394camera_list_t * list;
    dc1394camera_t *cam = NULL;
    uint32_t generation, latch=0;
    uint32_t node;
    ELLLIST camList;
    camNode_t *camListItem, *tmp;
    unsigned int i, newBus;

    d = dc1394_new ();
    ERR( dc1394_camera_enumerate (d, &list) );
    printf("Found %d cameras\n", list->num);

    // To reset a multi-bus system it is necessary to find a camera on each
    // individual bus and call the reset function with that camera.
    ellInit(&camList);

    // Get the 'generation' parameter for each camera. This parameter indicate
    // which bus the camera is located on.
    // For each specific 'generation' we add the camera handle to a list which
    // we can later use to reset each bus.
    for (i=0;i<list->num; i++)
    {
        printf("cam ID: %16.16llX", list->ids[i].guid);
        fflush(stdout);
        cam = dc1394_camera_new (d, list->ids[i].guid);
        DEBUG( dc1394_camera_get_node(cam, &node, &generation) );
        printf("  busID=%d\n", generation);

        // Run through the collected list of cameras and check if anyone
        // has the same 'generation' parameter... (i.e. is on the same bus)
        tmp=(camNode_t*)ellFirst(&camList);
        newBus = 1;
        while(tmp!=NULL)
        {
            if (generation == tmp->generation)
            {
                newBus = 0;
                break;
            }
            tmp=(camNode_t*)ellNext((ELLNODE*)tmp);
        }

        // If we havent already listed a camera on this bus -or if this is the
        // first camera we check: add the camera handle to a list of cameras that
        // we want to use for resetting busses.
        // Else free up the camera handle as we won't use it until we instantiate
        // our driver plugin.
        if (newBus==1 || i==0)
        {
            camListItem = (camNode_t*)calloc(1, sizeof(camNode_t));
            camListItem->cam = cam;
            camListItem->generation = generation;
            ellAdd(&camList, (ELLNODE*)camListItem);
            latch = generation;
        } else
        {
            // if we dont need the camera handle to reset the bus
            // we might as well free it up
            dc1394_camera_free(cam);
        }
    }

    // Go through the list of cameras that have been identified to be
    // on separate physical busses. Call reset for each of them and free
    // up the camera handle
    camListItem = (camNode_t*)ellFirst(&camList);
    while(camListItem != NULL)
    {
        printf("Resetting bus: %3d using cam: 0x%16.16llX... ", camListItem->generation, camListItem->cam->guid);
        fflush(stdout);
        DEBUG( dc1394_reset_bus(camListItem->cam) );
        printf("Done\n");
        dc1394_camera_free(camListItem->cam);
        camListItem = (camNode_t*)ellNext((ELLNODE*)camListItem);
    }

    // Clear up after ourselves.
    ellFree(&camList);
    dc1394_camera_free_list (list);
    dc1394_free (d);
    printf("\n");
    return;
}
#define NFRAMES 100
int main(int argc,char *argv[])
{
    unsigned int i, frames[10], todo;
    dc1394camera_t *cams[10];
    dc1394video_frame_t * dc1394_frame;

    for (int reps = 0; reps<10000; reps++) {
        printf("Rep: %d\n", reps);
        if (dc1394camList!=NULL) dc1394_camera_free_list(dc1394camList);
        if (dc1394fwbus!=NULL) dc1394_free(dc1394fwbus);

        // First reset the bus
        reset_bus();

        /* initialise the bus */
        dc1394fwbus = dc1394_new ();

        /* scan the bus for all cameras */
        DEBUG(dc1394_camera_enumerate (dc1394fwbus, &dc1394camList));

        /* initialise each camera */
        /* configure the camera to the mode and so on... */
        for (i = 0; i < dc1394camList->num; i++) {
            /* initialise the camera on the bus */
            cams[i] = dc1394_camera_new (dc1394fwbus, dc1394camList->ids[i].guid);        
            printf("%p = dc1394_camera_new(%p, %llX)\n", cams[i], dc1394fwbus, dc1394camList->ids[i].guid);                
        }

        /* start capture */
        for (i = 0; i < 3; i++) {
            DEBUG(dc1394_video_set_operation_mode(cams[i], DC1394_OPERATION_MODE_1394B));
            DEBUG(dc1394_video_set_iso_speed(cams[i], DC1394_ISO_SPEED_800));
            DEBUG(dc1394_video_set_mode(cams[i], DC1394_VIDEO_MODE_FORMAT7_MIN));
            DEBUG(dc1394_format7_set_color_coding(cams[i], DC1394_VIDEO_MODE_FORMAT7_MIN, DC1394_COLOR_CODING_MONO8));
            DEBUG(dc1394_feature_set_absolute_control(cams[i], DC1394_FEATURE_FRAME_RATE, DC1394_ON));
            DEBUG(dc1394_feature_set_absolute_value (cams[i], DC1394_FEATURE_FRAME_RATE, 25.0));
            frames[i] = 0;        
            DEBUG(dc1394_capture_setup(cams[i],FDC_DC1394_NUM_BUFFERS, DC1394_CAPTURE_FLAGS_DEFAULT));
            DEBUG(dc1394_video_set_transmission(cams[i], DC1394_ON));
        }

        /* grab NFRAMES frames */
        todo = 1;
        while (todo) {
            todo = 0;
            for (i = 0; i < 3; i++) {
                /* see of we need more frames */
                if (frames[i] >= NFRAMES) {
                    /* stop camera if we've got 10 frames */
                    if (frames[i] == NFRAMES) {
                        DEBUG(dc1394_video_set_transmission(cams[i], DC1394_OFF));
                        DEBUG(dc1394_capture_stop(cams[i]));
/*                        DEBUG(dc1394_iso_release_all(cams[i]));*/
                        frames[i]++;
                    }
                    continue;
                } else todo = 1;

                ERR(dc1394_capture_dequeue(cams[i], DC1394_CAPTURE_POLICY_POLL, &dc1394_frame));
                if (dc1394_frame) {
                    printf("%3d: Frame %d from cam %d\n", __LINE__, frames[i]++, i);
                    DEBUG(dc1394_capture_enqueue(cams[i], dc1394_frame));
                }
            }
        }
        
        /* free cams */
        for (i = 0; i < dc1394camList->num; i++) {        
            dc1394_camera_free(cams[i]);
        }
        
    }

    printf("All done\n");
}
