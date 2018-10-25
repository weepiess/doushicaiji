#include <iostream>
#include "auto_aim.h"
#include "usb_capture_with_thread.h"
#include "buff.h"


int main(int agrc, char *argv[])
{
    Buff buff;
    Mat image,mask;
    image=imread("/home/weepies/c++/buff0.0/a/doushicaiji-master/res/picture/s-r/75-12.jpg");
    buff.setImage(image,mask);
    buff.findcontours(mask,image);
    waitKey(0);
    return 0;
}
