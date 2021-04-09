//Convert Mat to Array
Mat filter(Mat input){
    if (input.empty()){
        cout << "Error: missing image\n\n" << endl;
    }
    //Convert image
    Mat output, kernel;
    cvtColor(input, output, COLOR_RGB2GRAY);

    /*
     *  Make binary image
     *
     *  threshold(src, dst, min, max)
     */
    threshold(output, output, 210, 255, THRESH_BINARY);

    /*  
     *  Closing morphology, removes small holes
     *  
     *  Closing = erode(dilute(src, dst, kernel))
     */
    kernel = getStructuringElement(MORPH_RECT, Size(5,5));
    morphologyEx(output, output, MORPH_CLOSE, kernel);

    /*  
     *  Opening morphology, removes small objects
     *  
     *  Opening = dilate(erode(src, dst, kernel))
     */
    kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    morphologyEx(output, output, MORPH_OPEN, kernel);

    /*
     *  Blurring, smooth the edges
     *
     *  GaussianBlur(src, dst, kernel size, sigma)
     *      sigma - how much image blur
     */
    GaussianBlur(output, output, Size(3,3), 1);

    return output;
}