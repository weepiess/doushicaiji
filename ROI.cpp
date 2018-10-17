int getROI(Mat &image, Rect &rect)
{
    Mat img=image.clone();
    Mat roi;
    int cols=img.cols, rows=img.rows;
    if(cols-1-rect.x<rect.width||rows-1-rect.y<rect.height)
        return -1;
    roi=img(Rect(rect.x, rect.y, rect.width, rect.height));
    rectangle(img, rect, Scalar(0, 0, 255),2);
    imshow("SignROI",img);
    image.copyTo(img);    
    imshow("ROI",roi);
}

