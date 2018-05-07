/// @file   anms.cpp
/// @author Bailo, Oleksandr and Rameau, Francois and Joo, Kyungdon and Park, Jinsun and Bogdan, Oleksandr and Kweon, In So
///         adapted by Marie-Neige Chapel
/// @date   2018/05/03



#include <component/featureDetector/anmsDetector.h>



//---------------------------------------------------------------------------------------
std::vector<cv::KeyPoint> AnmsDetector::brownANMS(std::vector<cv::KeyPoint> key_points, int nb_points)
{
    std::vector<cv::KeyPoint> kp;
    std::vector<std::pair<float,int>> results;
    results.push_back(std::make_pair(FLT_MAX,0));

    uint nb_key_points = key_points.size();
    for(uint i=1; i<nb_key_points; ++i) //for every keyPoint we get the min distance to the previously visited keyPoints
    {
        float minDist = FLT_MAX;
        for(uint j=0; j<i; ++j)
        {
            float exp1 = (key_points[j].pt.x-key_points[i].pt.x);
            float exp2 = (key_points[j].pt.y-key_points[i].pt.y);
            float curDist = sqrt(exp1*exp1+exp2*exp2);
            minDist = std::min(curDist,minDist);
        }
        results.push_back(std::make_pair(minDist,i));
    }

    // Sort by radius
    std::sort(results.begin(), results.end(), [](const std::pair<float,int>& left, const std::pair<float,int>& right)
        {
            return left.first > right.first;
        }
    );

    for(int i=0;i<nb_points;++i)
        kp.push_back(key_points[results[i].second]); //extracting numRetPoints keyPoints

    return kp;
}



//---------------------------------------------------------------------------------------
void AnmsDetector::compute() noexcept
{
    std::vector<cv::Point2f> feature_points;
    std::vector<cv::KeyPoint> key_points;
    int threshold = 20;
    int nb_points = 750; // the exact number of key points
    float tolerance = 0.1;

    cv::Mat img = data->getImageAtT();
    cv::cvtColor(img, img, CV_RGB2GRAY);

    cv::FAST(img, key_points, threshold, true);
    sortKeyPointsByResponse(key_points);
//    std::vector<cv::KeyPoint> topn_key_points = topN(key_points, nb_points);
//    std::vector<cv::KeyPoint> brown_key_points = brownANMS(key_points, nb_points);
//    std::vector<cv::KeyPoint> sdc_key_points = sdc(key_points, nb_points, tolerance, img.cols, img.rows);
//    std::vector<cv::KeyPoint> kdtreeKP = kdTree(key_points, nb_points, tolerance, img.cols, img.rows);
//    std::vector<cv::KeyPoint> rangetreeKP = RangeTree(key_points, nb_points, tolerance, img.cols, img.rows);

    std::vector<cv::KeyPoint> ssc_key_points = ssc(key_points, nb_points, tolerance, img.cols, img.rows);
    cv::KeyPoint::convert(ssc_key_points, feature_points);

    std::vector<cv::Mat> fp_to_add;
    bool to_add;
    uint nb_fp_candidates = feature_points.size();
    for(uint i=0; i<nb_fp_candidates; i++)
    {
        to_add = true;
        cv::Mat candidate((cv::Point2d)feature_points[i]);

        for(uint j=0; j<CST(int,cst::NB_MAX_POINT); j++)
        {
            if(!data->isFeaturePoint(j))
                continue;

            cv::Mat fp = *(data->getFeaturePointPosition2DAtT(j));

            if(cv::norm(candidate-fp)<10)
            {
                to_add = false;
                break;
            }
        }

        if(to_add)
            fp_to_add.push_back(candidate);
    }

    std::cout << "Nb new points " << fp_to_add.size() << std::endl;
    for(auto& fp: fp_to_add)
    {
        cv::Mat point(fp);
        data->addNewFeaturePoint(point);
    }

#ifndef NDEBUG
    DataWriter::writeFeaturePointPositionAge(data, getComponentName());
    DataWriter::paintFeaturePoint(data, getComponentName());
#endif
}



//---------------------------------------------------------------------------------------
std::string AnmsDetector::getComponentName() const noexcept
{
    return "AnmsDetector";
}



//---------------------------------------------------------------------------------------
bool AnmsDetector::isEnoughTimeElapsed() const noexcept
{
    return true;
}



//---------------------------------------------------------------------------------------
std::vector<cv::KeyPoint> AnmsDetector::kdTree(std::vector<cv::KeyPoint> key_points, int nb_points, float tolerance, int cols, int rows)
{
    // several temp expression variables to simplify solution equation
    int exp1 = rows + cols + 2*nb_points;
    long long exp2 = ((long long) 4*cols + (long long)4*nb_points + (long long)4*rows*nb_points
                    + (long long)rows*rows + (long long) cols*cols - (long long)2*rows*cols
                    + (long long)4*rows*cols*nb_points);
    double exp3 = sqrt(exp2);
    double exp4 = (2*(nb_points - 1));

    double sol1 = -round((exp1+exp3)/exp4); // first solution
    double sol2 = -round((exp1-exp3)/exp4); // second solution

    int high = (sol1>sol2)? sol1 : sol2; //binary search range initialization with positive solution
    int low  = floor(sqrt((double)key_points.size()/nb_points));


    PointCloud<int> cloud; //creating k-d tree with keypoints
    generatePointCloud(cloud, key_points);
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<int, PointCloud<int>> , PointCloud<int>, 2> my_kd_tree_t;
    my_kd_tree_t index(2, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(25 /* max leaf */) );
    index.buildIndex();

    bool complete = false;
    unsigned int K = nb_points; unsigned int Kmin = round(K-(K*tolerance)); unsigned int Kmax = round(K+(K*tolerance));
    std::vector<int> result_vec;
    int radius; int prevradius = -1;

    std::vector<int> result;
    result.reserve(key_points.size());

    while(!complete){
        std::vector<bool> included(key_points.size(),true);
        radius = low+(high-low)/2;
        if(radius == prevradius || low>high) //needed to reassure the same radius is not repeated again
        {
            result_vec = result; //return the keypoints from the previous iteration
            break;
        }
        result.clear();

        for (unsigned int i=0;i<key_points.size();++i){
            if (included[i]==true){
                included[i] = false;
                result.push_back(i);
                const int search_radius = static_cast<int>(radius*radius);
                std::vector<std::pair<size_t,int>> ret_matches;
                nanoflann::SearchParams params;
                const int query_pt[2] = {(int)key_points[i].pt.x, (int)key_points[i].pt.y};
                const size_t nMatches = index.radiusSearch(&query_pt[0], search_radius, ret_matches, params);

                for(size_t nmIdx=0; nmIdx<nMatches; nmIdx++)
                {
                    if(included[ret_matches[nmIdx].first])
                        included[ret_matches[nmIdx].first] = false;
                }
            }
        }

        if(result.size()>=Kmin && result.size()<=Kmax) //solution found
        {
            result_vec = result;
            complete = true;
        }
        else if(result.size()<Kmin) high = radius-1; //update binary search range
        else low = radius+1;

        prevradius = radius;
    }

    // retrieve final keypoints
    std::vector<cv::KeyPoint> kp;
    for(uint i=0; i<result_vec.size(); i++)
        kp.push_back(key_points[result_vec[i]]);

    return kp;
}



//---------------------------------------------------------------------------------------
void AnmsDetector::readFileData() noexcept
{

}



//---------------------------------------------------------------------------------------
std::vector<cv::KeyPoint> AnmsDetector::RangeTree(std::vector<cv::KeyPoint> key_points, int nb_points, float tolerance, int cols, int rows) noexcept
{
    // several temp expression variables to simplify solution equation
    int exp1 = rows + cols + 2*nb_points;
    long long exp2 = ((long long) 4*cols + (long long)4*nb_points + (long long)4*rows*nb_points
                    + (long long)rows*rows + (long long) cols*cols - (long long)2*rows*cols
                    + (long long)4*rows*cols*nb_points);
    double exp3 = sqrt(exp2);
    double exp4 = (2*(nb_points - 1));

    double sol1 = -round((exp1+exp3)/exp4); // first solution
    double sol2 = -round((exp1-exp3)/exp4); // second solution

    int high = (sol1>sol2)? sol1 : sol2; //binary search range initialization with positive solution
    int low = floor(sqrt((double)key_points.size()/nb_points));

    rangetree<u16, u16> treeANMS(key_points.size(), key_points.size()); //creating range tree with keypoints
    for(uint i = 0; i < key_points.size(); i++)
        treeANMS.add(key_points[i].pt.x, key_points[i].pt.y, (u16 *)(intptr_t)i);
    treeANMS.finalize();

    bool complete = false;
    uint K = nb_points;
    uint Kmin = round(K-(K*tolerance));
    uint Kmax = round(K+(K*tolerance));
    std::vector<int> result_vec;
    int width;
    int prevwidth = -1;

    std::vector<int> result;
    result.reserve(key_points.size());

    while(!complete){
        std::vector<bool> included(key_points.size(),true);
        width = low+(high-low)/2;
        if (width == prevwidth || low>high) { //needed to reassure the same width is not repeated again
            result_vec = result; //return the keypoints from the previous iteration
            break;
        }
        result.clear();

        for (unsigned int i=0;i<key_points.size();++i){
            if (included[i]==true){
                included[i] = false;
                result.push_back(i);
                int minx = key_points[i].pt.x-width;
                int maxx = key_points[i].pt.x+width; //defining square boundaries around the point
                int miny= key_points[i].pt.y-width;
                int maxy= key_points[i].pt.y+width;
                if (minx<0) minx = 0; if (miny<0) miny = 0;

                std::vector<u16 *> *he = treeANMS.search(minx, maxx, miny, maxy);
                for (uint j=0; j<he->size(); j++)
                    if (included[(u64) (*he)[j]])
                        included[(u64) (*he)[j]] = false;
                delete he;
                he = NULL;
            }
        }
        if (result.size()>=Kmin && result.size()<=Kmax){ //solution found
                result_vec = result;
                complete = true;
        }
        else if (result.size()<Kmin) high = width-1; //update binary search range
        else low = width+1;
        prevwidth = width;
    }
    // retrieve final keypoints
    std::vector<cv::KeyPoint> kp;
    for(uint i=0; i<result_vec.size(); i++)
        kp.push_back(key_points[result_vec[i]]);

    return kp;
}



//---------------------------------------------------------------------------------------
std::vector<cv::KeyPoint> AnmsDetector::sdc(std::vector<cv::KeyPoint> key_points, int nb_points, float tolerance, int cols, int rows) noexcept
{
    double eps_var = 0.25; //this parameter is chosen to be the most optimal in the original paper

    int low = 1; int high = cols; //binary search range initialization
    int radius;
    int prevradius = -1;

    std::vector<int> result_vec;
    bool complete = false;
    unsigned int K = nb_points;
    unsigned int Kmin = round(K-(K*tolerance));
    unsigned int Kmax = round(K+(K*tolerance));

    std::vector<int> result;
    result.reserve(key_points.size());

    while(!complete)
    {
        radius = low+(high-low)/2;
        if (radius == prevradius || low>high) //needed to reassure the same radius is not repeated again
        {
            result_vec = result; //return the keypoints from the previous iteration
            break;
        }

        result.clear();
        double c = eps_var*radius/sqrt(2); //initializing Grid
        int num_cell_cols = floor(cols/c);
        int num_cell_rows = floor(rows/c);
        std::vector<std::vector<bool>> covered_vec(num_cell_rows+1, std::vector<bool>(num_cell_cols+1,false));

        uint nb_key_points = key_points.size();
        for(uint i=0; i<nb_key_points; ++i)
        {
            int row = floor(key_points[i].pt.y/c); //get position of the cell current point is located at
            int col = floor(key_points[i].pt.x/c);
            if(covered_vec[row][col]==false) // if the cell is not covered
            {
                result.push_back(i);
                int rowMin = ((row-floor(radius/c))>=0)? (row-floor(radius/c)) : 0; //get range which current radius is covering
                int rowMax = ((row+floor(radius/c))<=num_cell_rows)? (row+floor(radius/c)) : num_cell_rows;
                int colMin = ((col-floor(radius/c))>=0)? (col-floor(radius/c)) : 0;
                int colMax = ((col+floor(radius/c))<=num_cell_cols)? (col+floor(radius/c)) : num_cell_cols;
                for(int rowToCov=rowMin; rowToCov<=rowMax; ++rowToCov)
                {
                    for(int colToCov=colMin ; colToCov<=colMax; ++colToCov)
                    {
                        double dist = sqrt((rowToCov-row)*(rowToCov-row) + (colToCov-col)*(colToCov-col));
                        if(dist<=((double)radius)/c) covered_vec[rowToCov][colToCov] = true; //check the distance to every cell
                    }
                }
            }
        }

        if(result.size()>=Kmin && result.size()<=Kmax) //solution found
        {
            result_vec = result;
            complete = true;
        }
        else if(result.size()<Kmin)
            high = radius-1; //update binary search range
        else low = radius+1;
    }

    // retrieve final keypoints
    std::vector<cv::KeyPoint> kp;
    uint result_vec_size = result_vec.size();
    for(uint i = 0; i<result_vec_size; i++)
        kp.push_back(key_points[result_vec[i]]);

    return kp;
}



//---------------------------------------------------------------------------------------
void AnmsDetector::sortKeyPointsByResponse(std::vector<cv::KeyPoint>& key_points) noexcept
{
    std::vector<int> response_vector;
    uint nb_key_points = key_points.size();
    for(uint i=0 ; i<nb_key_points; i++)
        response_vector.push_back(key_points[i].response);

    std::vector<int> indx(response_vector.size());
    std::iota(std::begin(indx), std::end(indx), 0);
    cv::sortIdx(response_vector, indx, CV_SORT_DESCENDING);
    std::vector<cv::KeyPoint> key_points_sorted;
    for (uint i=0; i<key_points.size(); i++)
        key_points_sorted.push_back(key_points[indx[i]]);
}



//---------------------------------------------------------------------------------------
std::vector<cv::KeyPoint> AnmsDetector::ssc(std::vector<cv::KeyPoint> key_points, int nb_points,float tolerance, int cols, int rows) noexcept
{
    // several temp expression variables to simplify solution equation
    int exp1 = rows + cols + 2*nb_points;
    long long exp2 = ((long long) 4*cols + (long long)4*nb_points + (long long)4*rows*nb_points + (long long)rows*rows
                    + (long long) cols*cols - (long long)2*rows*cols + (long long)4*rows*cols*nb_points);
    double exp3 = sqrt(exp2);
    double exp4 = (2*(nb_points - 1));

    double sol1 = -round((exp1+exp3)/exp4); // first solution
    double sol2 = -round((exp1-exp3)/exp4); // second solution

    int high = (sol1>sol2)? sol1 : sol2; //binary search range initialization with positive solution
    int low = floor(sqrt((double)key_points.size()/nb_points));

    int width;
    int prevWidth = -1;

    std::vector<int> result_vec;
    bool complete = false;
    uint K = nb_points;
    uint Kmin = round(K-(K*tolerance));
    uint Kmax = round(K+(K*tolerance));

    std::vector<int> result;
    result.reserve(key_points.size());

    while(!complete)
    {
        width = low+(high-low)/2;
        if (width == prevWidth || low>high) //needed to reassure the same radius is not repeated again
        {
            result_vec = result; //return the keypoints from the previous iteration
            break;
        }
        result.clear();
        double c = width/2; //initializing Grid
        int numCellCols = floor(cols/c);
        int numCellRows = floor(rows/c);
        std::vector<std::vector<bool>> covered_vec(numCellRows+1,std::vector<bool>(numCellCols+1,false));

        for(uint i=0; i<key_points.size(); ++i)
        {
            int row = floor(key_points[i].pt.y/c); //get position of the cell current point is located at
            int col = floor(key_points[i].pt.x/c);
            if(covered_vec[row][col]==false) // if the cell is not covered
            {
                result.push_back(i);
                int rowMin = ((row-floor(width/c))>=0)? (row-floor(width/c)) : 0; //get range which current radius is covering
                int rowMax = ((row+floor(width/c))<=numCellRows)? (row+floor(width/c)) : numCellRows;
                int colMin = ((col-floor(width/c))>=0)? (col-floor(width/c)) : 0;
                int colMax = ((col+floor(width/c))<=numCellCols)? (col+floor(width/c)) : numCellCols;
                for(int rowToCov=rowMin; rowToCov<=rowMax; ++rowToCov)
                {
                    for(int colToCov=colMin ; colToCov<=colMax; ++colToCov)
                    {
                        if(!covered_vec[rowToCov][colToCov])
                            covered_vec[rowToCov][colToCov] = true; //cover cells within the square bounding box with width w
                    }
                }
            }
        }

        if(result.size()>=Kmin && result.size()<=Kmax) //solution found
        {
            result_vec = result;
            complete = true;
        }
        else if(result.size()<Kmin)
            high = width-1; //update binary search range
        else low = width+1;
        prevWidth = width;
    }
    // retrieve final keypoints
    std::vector<cv::KeyPoint> kp;
    for(uint i=0; i<result_vec.size(); i++)
        kp.push_back(key_points[result_vec[i]]);

    return kp;
}



//---------------------------------------------------------------------------------------
std::vector<cv::KeyPoint> AnmsDetector::topN(std::vector<cv::KeyPoint> key_points, int nb_points) noexcept
{
    std::vector<cv::KeyPoint> kp;
    for (int i=0; i<nb_points; i++)
        kp.push_back(key_points[i]); //simply extracting numRetPoints keyPoints

    return kp;
}
