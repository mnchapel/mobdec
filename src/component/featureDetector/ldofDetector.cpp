/// @file   ldofDetector.cpp
/// @author Marie-Neige Chapel
/// @date   2017/10/18



#include <component/featureDetector/ldofDetector.h>



//---------------------------------------------------------------------------------------
void LdofOptF::compute() noexcept
{
    convertImageToPPM();
    computeOpticalFlow(CST(int,cst::LDOF_SUBSAMPLING_FACTOR));

#ifndef NDEBUG
//    if(data->getTimeElapsed() >= OPTICAL_FLOW_AGE) // TODO
//        result_file.paintOpticalFlowTTMDeltaOnCurrentImage(data, getComponentName()); // TODO
    DataWriter::writeFeaturePointPositionAge(data, getComponentName());
    DataWriter::paintFeaturePoint(data, getComponentName());
    DataWriter::paintOpticalFlow(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string LdofOptF::getComponentName() const noexcept
{
    return "LdofOptF";
}



//---------------------------------------------------------------------------------------
void LdofOptF::computeCorners(CTensor<float>& aImage, CMatrix<float>& aCorners, float aRho) noexcept
{
    aCorners.setSize(aImage.xSize(),aImage.ySize());
    int aXSize = aImage.xSize();
    int aYSize = aImage.ySize();
    int aSize = aXSize*aYSize;

    // Compute gradient
    CTensor<float> dx(aXSize,aYSize,aImage.zSize());
    CTensor<float> dy(aXSize,aYSize,aImage.zSize());
    CDerivative<float> aDerivative(3);
    NFilter::filter(aImage,dx,aDerivative,1,1);
    NFilter::filter(aImage,dy,1,aDerivative,1);

    // Compute second moment matrix
    CMatrix<float> dxx(aXSize,aYSize,0);
    CMatrix<float> dyy(aXSize,aYSize,0);
    CMatrix<float> dxy(aXSize,aYSize,0);

    int i2 = 0;
    for (int k=0; k<aImage.zSize(); k++)
        for (int i=0; i<aSize; i++, i2++)
        {
            dxx.data()[i] += dx.data()[i2]*dx.data()[i2];
            dyy.data()[i] += dy.data()[i2]*dy.data()[i2];
            dxy.data()[i] += dx.data()[i2]*dy.data()[i2];
        }

    // Smooth second moment matrix
    NFilter::recursiveSmoothX(dxx,aRho);
    NFilter::recursiveSmoothY(dxx,aRho);
    NFilter::recursiveSmoothX(dyy,aRho);
    NFilter::recursiveSmoothY(dyy,aRho);
    NFilter::recursiveSmoothX(dxy,aRho);
    NFilter::recursiveSmoothY(dxy,aRho);

    // ComputeCorners smallest eigenvalue
    for (int i = 0; i < aSize; i++)
    {
        float a = dxx.data()[i];
        float b = dxy.data()[i];
        float c = dyy.data()[i];
        float temp = 0.5*(a+c);
        float temp2 = temp*temp+b*b-a*c;

        if(temp2 < 0.0f)
            aCorners.data()[i] = 0.0f;
        else
            aCorners.data()[i] = temp-sqrt(temp2);
    }
}



//---------------------------------------------------------------------------------------
void LdofOptF::convertImageToPPM() noexcept
{
    cv::Mat image_t = data->getImageAtT();
    cv::Mat image_tm1;

    if(data->getTimeElapsed() == 0)
        image_tm1 = data->getImageAtT();
    else
        image_tm1 = data->getImageAtTM1();

    cv::imwrite("image_t.ppm", image_t);
    cv::imwrite("image_tm1.ppm", image_tm1);
}



//---------------------------------------------------------------------------------------
/// Code fragment from Pedro Felzenszwalb
void LdofOptF::dt(CVector<float>& f, CVector<float>& d, int n) noexcept
{
    d.setSize(n);
    int *v = new int[n];
    float *z = new float[n+1];
    int k = 0;
    v[0] = 0;
    z[0] = -10e20;
    z[1] = 10e20;
    for (int q = 1; q <= n-1; q++)
    {
        float s  = ((f[q]+q*q)-(f(v[k])+v[k]*v[k]))/(2*(q-v[k]));
        while (s <= z[k])
        {
            k--;
            s  = ((f[q]+q*q)-(f(v[k])+v[k]*v[k]))/(2*(q-v[k]));
        }

        k++;
        v[k] = q;
        z[k] = s;
        z[k+1] = 10e20;
    }

    k = 0;
    for (int q = 0; q <= n-1; q++)
    {
        while (z[k+1] < q)
          k++;

        int help = q-v[k];
        d(q) = help*help + f(v[k]);
    }

    delete[] v;
    delete[] z;
}



//---------------------------------------------------------------------------------------
void LdofOptF::euclideanDistanceTransform(CMatrix<float>& aMatrix) noexcept
{
    int aXSize = aMatrix.xSize();
    int aYSize = aMatrix.ySize();
    CVector<float> f(NMath::max(aXSize,aYSize));

    // Transform along columns
    for (int x = 0; x < aXSize; x++)
    {
        for (int y = 0; y < aYSize; y++)
          f(y) = aMatrix(x,y);

        CVector<float> d;
        dt(f,d,aYSize);

        for (int y = 0; y < aYSize; y++)
          aMatrix(x,y) = d(y);
    }

    // Transform along rows
    for (int y = 0; y < aYSize; y++)
    {
        int aOffset = y*aXSize;
        for (int x = 0; x < aXSize; x++)
          f(x) = aMatrix.data()[x+aOffset];

        CVector<float> d;
        dt(f,d,aXSize);

        for (int x = 0; x < aXSize; x++)
          aMatrix.data()[x+aOffset] = d(x);
    }
}



//---------------------------------------------------------------------------------------
bool LdofOptF::isEnoughTimeElapsed() const noexcept
{
	return true;
}



//---------------------------------------------------------------------------------------
void LdofOptF::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
bool LdofOptF::readMiddlebury(const char* aFilename, CTensor<float>& aFlow) noexcept
{
    FILE *stream = fopen(aFilename, "rb");
    if (stream == 0)
    {
        std::cerr << "\t\tCould not open " << aFilename << std::endl;
        return false;
    }

    float help;
    int dummy;
    dummy = fread(&help,sizeof(float),1,stream);
    int aXSize,aYSize;
    dummy = fread(&aXSize,sizeof(int),1,stream);
    dummy = fread(&aYSize,sizeof(int),1,stream);
    aFlow.setSize(aXSize,aYSize,2);
    for (int y = 0; y < aFlow.ySize(); y++)
        for (int x = 0; x < aFlow.xSize(); x++)
        {
            dummy = fread(&aFlow(x,y,0),sizeof(float),1,stream);
            dummy = fread(&aFlow(x,y,1),sizeof(float),1,stream);
        }

    fclose(stream);

    return true;
}



//---------------------------------------------------------------------------------------
bool LdofOptF::writeMiddlebury(const char* aFilename, CTensor<float>& aFlow) noexcept
{
    FILE *stream = fopen(aFilename, "wb");
    if (stream == 0)
    {
        std::cerr << "\tCould not open... " << aFilename << std::endl;
        return false;

    }

    float help=202021.25;
    int dummy;
    dummy = fwrite(&help,sizeof(float),1,stream);
    int aXSize = aFlow.xSize();
    int aYSize = aFlow.ySize();
    fwrite(&aXSize,sizeof(int),1,stream);
    fwrite(&aYSize,sizeof(int),1,stream);

    //aFlow.setSize(aXSize,aYSize,2);
    for (int y = 0; y < aFlow.ySize(); y++)
        for (int x = 0; x < aFlow.xSize(); x++)
        {
            fwrite(&aFlow(x,y,0),sizeof(float),1,stream);
            fwrite(&aFlow(x,y,1),sizeof(float),1,stream);
        }
    fclose(stream);

    return true;
}



//---------------------------------------------------------------------------------------
void LdofOptF::updateTracks(uint size_x,
                            uint size_y,
                            CMatrix<float> &unreliable,
                            CTensor<float> &forward) noexcept
{
    uint count = 0;
    for(unsigned int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
    {
        if(!data->isFeaturePoint(i))
        	continue;

        float ax, ay;
        ax = data->getFeaturePointPosition2DAtTM1(i).at<double>(0);
        ay = data->getFeaturePointPosition2DAtTM1(i).at<double>(1);

        int iax = lroundf(ax);
        int iay = lroundf(ay);

        // Can't LdofOptF the point anymore.
        if(unreliable(iax,iay) > 0)
        {
            data->deleteFeaturePoint(i);
            continue;
        }

        float bx = ax+forward(iax,iay,0);
        float by = ay+forward(iax,iay,1);
        int ibx = lroundf(bx);
        int iby = lroundf(by);

        // The point is out of the image
        if(ibx < 0 || iby < 0 || ibx >= size_x || iby >= size_y)
        {
            data->deleteFeaturePoint(i);
            continue;
        }

        cv::Mat feature_point(2,1,CV_64F);
        feature_point.at<double>(0) = bx;
        feature_point.at<double>(1) = by;
        data->updateFeaturePoint2DPosition(i, feature_point);
        count++;
    }

    DEBUG_COMPONENT_MSG("count up " << count);
}



//---------------------------------------------------------------------------------------
void LdofOptF::computeOpticalFlow(const unsigned int subsampling_factor) noexcept
{
    CTensor<float>* image1;
    CTensor<float>* image2;

    char buffer[500];

    // Load next image
    image1 = new CTensor<float>;
    image1->readFromPPM("image_tm1.ppm");

    image2 = new CTensor<float>;
    image2->readFromPPM("image_t.ppm");

    uint size_x = image1->xSize();
    uint size_y = image1->ySize();
    uint size = size_x*size_y;

    CMatrix<float> corners;
    CMatrix<float> covered(size_x, size_y);

    NFilter::recursiveSmoothX(*image2,0.8f);
    NFilter::recursiveSmoothY(*image2,0.8f);

    // Mark areas sufficiently covered by LdofOptFs
    covered = 1e20;

    if(data->getTime() > 0)
    {
        for(int i=0; i<CST(int,cst::NB_MAX_POINT); i++)
        {
            if(!data->isFeaturePoint(i))
                continue;

            covered((int)data->getFeaturePointPosition2DAtTM1(i).at<double>(0),
                    (int)data->getFeaturePointPosition2DAtTM1(i).at<double>(1)) = 0.0f;
        }

        euclideanDistanceTransform(covered);
    }


    // Set up new LdofOptFing points in uncovered areas
    computeCorners(*image1, corners,3.0f);

    std::vector<cv::Mat> new_candidate_point;
    float cornerAvg = corners.avg();
    for (int ay = 4; ay < size_y-4; ay+=subsampling_factor)
        for (int ax = 4; ax < size_x-4; ax+=subsampling_factor)
        {
            if (covered(ax, ay) < subsampling_factor*subsampling_factor) continue;
            float distToImageBnd = exp(-0.1*NMath::min(NMath::min(NMath::min(ax,ay), size_x-ax), size_y-ay));
            if (corners(ax, ay) < 1.0* (cornerAvg*(0.1+distToImageBnd)))
                continue;
            if (corners(ax, ay) < 1.0*(1.0f+distToImageBnd))
                continue;

            cv::Mat feature_point = cv::Mat::zeros(2,1,CV_64F);
            feature_point.at<double>(0) = ax;
            feature_point.at<double>(1) = ay;
            new_candidate_point.push_back(feature_point);
        }

    // Compute bidirectional LDOF or read from file when available
    CTensor<float> forward, backward;
    sprintf(buffer,"ForwardFlow%03d.flo",data->getTime());
    std::string name1 = CST(std::string,cst::MOBDEC_DATA_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + getComponentName() + "/" + buffer;
    sprintf(buffer,"BackwardFlow%03d.flo",data->getTime());
    std::string name2 = CST(std::string,cst::MOBDEC_DATA_PATH) + "/" + CST(std::string,cst::PROJECT_NAME) + "/" + getComponentName() + "/" + buffer;

    if(readMiddlebury(name1.c_str(), forward) == false || readMiddlebury(name2.c_str(), backward) == false)
    {
        // Call optical flow
        ldof(*image1, *image2, forward, backward);
        writeMiddlebury(name1.c_str(), forward);
        writeMiddlebury(name2.c_str(), backward);
    }

    // Check consistency of forward flow via backward flow
    CMatrix<float> unreliable(size_x, size_y, 0);
    CTensor<float> dx(size_x, size_y, 2);
    CTensor<float> dy(size_x, size_y, 2);
    CDerivative<float> dev(3);
    NFilter::filter(forward, dx, dev, 1, 1);
    NFilter::filter(forward, dy, 1, dev, 1);
    CMatrix<float> motionEdge(size_x, size_y, 0);
    for (int i=0; i<size; i++)
    {
        motionEdge.data()[i] += dx.data()[i]*dx.data()[i];
        motionEdge.data()[i] += dx.data()[size+i]*dx.data()[size+i];
        motionEdge.data()[i] += dy.data()[i]*dy.data()[i];
        motionEdge.data()[i] += dy.data()[size+i]*dy.data()[size+i];
    }


    for(int ay=0; ay<forward.ySize(); ay++)
        for(int ax=0; ax<forward.xSize(); ax++)
        {
            float bx = ax+forward(ax,ay,0);
            float by = ay+forward(ax,ay,1);
            int x1 = floor(bx);
            int y1 = floor(by);
            int x2 = x1+1;
            int y2 = y1+1;
            if (x1 < 0 || x2 >= size_x || y1 < 0 || y2 >= size_y)
            {
                unreliable(ax,ay) = 1.0f;
                continue;
            }

            float alphaX = bx-x1; float alphaY = by-y1;
            float a = (1.0-alphaX)*backward(x1,y1,0)+alphaX*backward(x2,y1,0);
            float b = (1.0-alphaX)*backward(x1,y2,0)+alphaX*backward(x2,y2,0);
            float u = (1.0-alphaY)*a+alphaY*b;
            a = (1.0-alphaX)*backward(x1,y1,1)+alphaX*backward(x2,y1,1);
            b = (1.0-alphaX)*backward(x1,y2,1)+alphaX*backward(x2,y2,1);
            float v = (1.0-alphaY)*a+alphaY*b;
            float cx = bx+u;
            float cy = by+v;
            float u2 = forward(ax,ay,0);
            float v2 = forward(ax,ay,1);

            if (((cx-ax)*(cx-ax)+(cy-ay)*(cy-ay)) >= 0.01*(u2*u2+v2*v2+u*u+v*v)+0.5f)
                unreliable(ax,ay) = 1.0f; continue;
            if (motionEdge(ax,ay) > 0.01*(u2*u2+v2*v2)+0.002f)
                unreliable(ax,ay) = 1.0f; continue;
        }

    if(data->getTimeElapsed()>0)
        updateTracks(size_x, size_y, unreliable, forward);

    uint count=0;
    bool to_add;
    for(int i=0; i<new_candidate_point.size(); i++)
    {
        to_add = true;
        cv::Mat candidate = new_candidate_point[i];

        for(int j=0; j<CST(int,cst::NB_MAX_POINT); j++)
        {
            if(!data->isFeaturePoint(j))
                continue;

            cv::Mat feature_point = *(data->getFeaturePointPosition2DAtT(j));

            if(cv::norm(feature_point-candidate) < subsampling_factor)
            {
                to_add = false;
                break;
            }
        }


        if(to_add)
        {
            data->addNewFeaturePoint(candidate);
            count++;
        }
    }

	DEBUG_COMPONENT_MSG("count new " << count);

    delete image1;
    delete image2;
}














