#include <iostream>
#include <fstream>
#include <string>
#include <sys/stat.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include "ardrone_autonomy/Navdata.h"
#include "/home/caro/fuerte_workspace/sandbox/tum_ardrone/msg_gen/cpp/include/tum_ardrone/filter_state.h"

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/util.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"

using namespace std;
using namespace cv;
using namespace cv::detail;

#define DESCRIPTOR      "BRISK"
#define DETECTOR	"FAST"

int imageready = 0, contador = 0;
bool Save_keypoints = true, Save_match = true;
char Directorio_Result[150], Directorio_Images[150];
namespace enc = sensor_msgs::image_encodings; 
cv_bridge::CvImagePtr cv_ptr;
vector<Mat> imgs;
Mat pano;

Ptr<FeatureDetector> detector = FeatureDetector::create(DETECTOR);
Ptr<DescriptorExtractor> descriptorExtractor=DescriptorExtractor::create(DESCRIPTOR);

void Image_Upload(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    imageready=1;
}

void Save_features_data (int indicador, ImageFeatures features, int i, char* save)
{
    FILE *dataError;
    char datalogError[512]; 
	if ((dataError=fopen(save,"a+"))!=NULL)
	    sprintf(datalogError,"%d;Posicion %d;%d", indicador, i, features.keypoints.size()/*, features[i].descriptors.size().width, features[i].descriptors.size().height*/);
    	fprintf(dataError,"%s \n", datalogError);
    	fclose(dataError);	
}

void Save_match_data (int indicador, MatchesInfo pairwise_matches, char* save)
{
    FILE *datanav;
    char datalog[512];

	if (pairwise_matches.src_img_idx != pairwise_matches.dst_img_idx)
	{
	    if ((datanav=fopen(save,"a+"))!=NULL)
	    {
	    	sprintf(datalog,"%d;%d;%d;%f;", indicador, pairwise_matches.num_inliers, pairwise_matches.matches.size(), pairwise_matches.confidence);
	    	fprintf(datanav,"%s", datalog);

		Mat HI = Mat::zeros(3,3,CV_32F);
	    	if (!pairwise_matches.H.empty())
		    HI = pairwise_matches.H.clone();
	    const double* h = reinterpret_cast<const double*>(HI.data);
	        for (int m = 0; m < 9; m++)
	        {
	    	    sprintf(datalog,"%f;", h[m]);
	     	    fprintf(datanav,"%s", datalog);
	    	}
	    }
 	    fprintf(datanav,"%s \n", datalog);
    	    fclose(datanav);
	}
}

void Save_camera_data (int indicador, vector<CameraParams> cameras, vector<double> Distancia_f)
{
    char save[150];
    FILE *satanav;
    char datalog[512];
    strcpy(save,Directorio_Result);
    strcpy(&save[ strlen(Directorio_Result) ], "/Camera.csv"); 
    for (int i = 0; i < cameras.size(); i++)
    {
	if ((satanav=fopen(save,"a+"))!=NULL)
	{
	    sprintf(datalog,"%d;Posicion %d;%f;%f;%f;%f;%f;", indicador, i, cameras[i].focal, cameras[i].ppx, cameras[i].ppy, Distancia_f[i], Distancia_f[i+3]);
	    fprintf(satanav,"%s", datalog);
	    for (int j = 0; j < 3; j++)
	    {
		sprintf(datalog,"%f,", cameras[i].t.at<float>(j));
		fprintf(satanav,"%s", datalog);
	    }
	    for (int j = 0; j < 3; j++){
	 	for (int m = 0; m < 3; m++)
		{
		    sprintf(datalog,"%f,", cameras[i].R.at<float>(j,m));
		    fprintf(satanav,"%s", datalog);
		}}
	}
    	fprintf(satanav,"%s \n", datalog);
    	fclose(satanav);
    }
    if ((satanav=fopen(save,"a+"))!=NULL)
    sprintf(datalog," \n");
    fprintf(satanav,"%s", datalog);
    fclose(satanav);			
}

void Detector_Descriptor(Mat img, cv::detail::ImageFeatures& featureimg)
{
    detector->detect( img, featureimg.keypoints);
    descriptorExtractor->compute( img, featureimg.keypoints, featureimg.descriptors );
    featureimg.img_size = img.size();
}

bool try_gpu = false;
double work_megapix = 0.6;
double seam_megapix = 0.1;
float match_conf = 0.3f;
float conf_thresh = 0.3f;
double compose_megapix = -1;
string ba_refine_mask = "xxxxx";
bool do_wave_correct = false /*true*/;
int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
int blend_type = Blender::/*MULTI_BAND*//*FEATHER*/NO;
float blend_strength = 5;
int grupo = 1;
Ptr<FeaturesFinder> finder = new OrbFeaturesFinder();
// /*"reproj"*/"ray"
Ptr<detail::BundleAdjusterBase> adjuster = new detail::BundleAdjusterRay();
// "plane"
Ptr<WarperCreator> warper_creator = new cv::PlaneWarper();
// "gc_color"
Ptr<SeamFinder> seam_finder = new detail::GraphCutSeamFinder(GraphCutSeamFinderBase::COST_COLOR);

void data_initialization (Mat_<uchar> &refine_mask)
{

    if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
    if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
    if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
    if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
    if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;

}

int feature_extraction (int contador, double &work_scale, double &seam_scale, double &seam_work_aspect, vector<ImageFeatures> &features, vector<Mat> &images, vector<Size> &full_img_sizes)
{
    char save[150];
    FILE *dataError;
    char datalogError[512];
    strcpy(save,Directorio_Result);
    strcpy(&save[ strlen(Directorio_Result) ], "/Features.csv");

    Mat img, IM_COLOR, full_img, IM_GRAY, outImg1;
    //double seam_work_aspect = 1, work_scale = 1, seam_scale = 1;
    bool is_work_scale_set = false, is_seam_scale_set = false;
    int num_images = static_cast<int>(imgs.size());

    for (int i = 0; i < num_images; ++i)
    {
	imgs[i].convertTo(imgs[i], CV_8U);
	IM_COLOR = imgs[i].clone();
	full_img = IM_GRAY.clone();
        full_img_sizes[i] = full_img.size();
        if (full_img.empty())
        {
            LOGLN("Can't open image ");
            return -1;
        }
        if (work_megapix < 0)
        {
            img = full_img;
            work_scale = 1;
            is_work_scale_set = true;
        }
        else
        {
            if (!is_work_scale_set)
            {
                work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
                is_work_scale_set = true;
            }
            resize(full_img, img, Size(), work_scale, work_scale);
            resize(IM_COLOR, IM_COLOR, Size(), work_scale, work_scale);
        }
        if (!is_seam_scale_set)
        {
            seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
            seam_work_aspect = seam_scale / work_scale;
            is_seam_scale_set = true;
        }

	img.convertTo(img, CV_8U);
	Detector_Descriptor(img, features[i]);
        features[i].img_idx = i;
        LOGLN("Features in image #" << i+1 << ": " << features[i].keypoints.size());
        resize(IM_COLOR, img, Size(), seam_scale, seam_scale);
        images[i] = img.clone();
    	Save_features_data (grupo, features[i], i, save);
	if (Save_keypoints == true)
	{
	    ostringstream os;
	    cvtColor(imgs[i], IM_GRAY, CV_RGB2GRAY);
    	    os << Directorio_Result << "/keypoints/out" << contador - (num_images - 1) + i << ".jpg";
	    drawKeypoints(img, features[i].keypoints, outImg1, Scalar(255,255,255), DrawMatchesFlags::DEFAULT);
    	    string result_name2 = os.str();
	    imwrite(result_name2, outImg1);
	}
	
    }
    cout << "images.size()" << images.size() << endl;
    finder->collectGarbage();
    full_img.release();
    img.release();

    if ((dataError=fopen(save,"a+"))!=NULL)
    	sprintf(datalogError," \n");
    fprintf(dataError,"%s", datalogError);
    fclose(dataError);
}

int Distancia (int contador, vector<int> &indices, vector<MatchesInfo> &pairwise_matches, vector<ImageFeatures> &features, vector<Mat> &images, vector<Size> full_img_sizes)
{
    float x=0, y=0, dx=0, dy=0, PM=0.8, Pm=0.2, fx = 7.0210540771484375e+02, fy = 6.9920452880859375e+02;
    int src=0, dst=0, num_images = static_cast<int>(images.size()), NumImg = 0;
    bool condicion = false;
    vector<int> posicion2(num_images); 
    vector<MatchesInfo> prueba_matches;
    vector<ImageFeatures> Prueba_features;
    Mat drawImg;

	/*if (grupo /*!=1== 0 && indices[0] != 0)
	    return 2;*/

    if (Save_match == true)
    {
	for (int i = 0; i < pairwise_matches.size(); i++)
	{
	    src = pairwise_matches[i].src_img_idx;
	    dst = pairwise_matches[i].dst_img_idx;
	    if (src != dst)
	    {
		ostringstream os;
    	    	os << Directorio_Result << "/match/out" << (contador*2) - ((num_images*2) - 1) + NumImg << ".jpg";
		NumImg++;
    		drawMatches( images[src], features[src].keypoints, images[dst], features[dst].keypoints, pairwise_matches[i].matches, drawImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255));
		string result_name2 = os.str();
		imwrite(result_name2, drawImg);
	     }
	 }
     } 

    vector<Mat> imgs_subset, images_subset;
    vector<Size> full_img_sizes_subset;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        images_subset.push_back(images[indices[i]]);
        imgs_subset.push_back(imgs[indices[i]]);
        full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
    }
    images = images_subset;
    imgs = imgs_subset;
    full_img_sizes = full_img_sizes_subset;
    return 1;
}

complex<double> calculo_f0 (Mat HI)
{
    const double* h = reinterpret_cast<const double*>(HI.data);
    complex<double> num = (h[5]*h[5]) - (h[2]*h[2]);
    complex<double> den = (h[0]*h[0]) + (h[1]*h[1]) - (h[3]*h[3]) - (h[4]*h[4]);
    return sqrt(num/den);
}

complex<double> calculo_f1 (Mat HI, complex<double> f0)
{
    const double* h = reinterpret_cast<const double*>(HI.data);
    //double num = (h[3]*h[3]) + (h[4]*h[4]) + (h[5]*h[5])/(f0*f0);
    complex<double> num = (h[0]*h[0]) + (h[1]*h[1]) + (h[2]*h[2])/(f0*f0);
    complex<double> den = (h[6]*h[6]) + (h[7]*h[7]) + (h[8]*h[8])/(f0*f0);
    return sqrt(num/den);
}

void Cambio_DF (vector<MatchesInfo> pairwise_matches, vector<double> &Distancia_f )
{
    complex<double> f1, f0;
    double f;
    int num_images = static_cast<int>(pairwise_matches.size());
    int num_images_B = static_cast<int>(Distancia_f.size()/2);
    int inicio = 1, m = 0;
    vector<double> DF(num_images);
    for (int i = 0; i < num_images; i++)
    {
	int src = pairwise_matches[i].src_img_idx;
	int dst = pairwise_matches[i].dst_img_idx;
	if (src != dst)
	{
	    f0 = calculo_f0(pairwise_matches[i].H);
	    f1 = calculo_f1(pairwise_matches[i].H, f0);
	    f = sqrt(abs(f0)*abs(f1));
	    DF[i] = f;
	}
    }
    for (int i = 0; i < num_images_B-1; i++)
    {
	for (int j = inicio; j < num_images_B; j++)
	{
	    int in = i*num_images_B+j;
	    int fi = j*num_images_B+i;
	    Distancia_f[m] = DF[in];
	    Distancia_f[m+3] = DF[fi];
	    m++;
	}
	inicio++;
    }
}

int panoramic()
{
    int64 t = getTickCount();
    cv::setBreakOnError(true);
    int num_images = static_cast<int>(imgs.size());
    if (num_images < 2)
    {
        LOGLN("Need more images");
        return 2;
    }

    Mat img, full_img;
    double work_scale = 1, seam_scale = 1, compose_scale = 1, seam_work_aspect = 1;
    bool is_compose_scale_set = false;

    vector<ImageFeatures> features(num_images);
    vector<Mat> images(num_images);
    vector<Size> full_img_sizes(num_images);
    vector<MatchesInfo> pairwise_matches;
    BestOf2NearestMatcher matcher(try_gpu, match_conf);

    int imt = feature_extraction (contador, work_scale, seam_scale, seam_work_aspect, features, images, full_img_sizes);
    cout << "Finaliza Caracteristicas" << images.size() << endl;
    matcher(features, pairwise_matches);
    matcher.collectGarbage();

    char Save[150];
    strcpy(Save,Directorio_Result);
    strcpy(&Save[ strlen(Directorio_Result) ], "/Match.csv");
    for (int i = 0; i < pairwise_matches.size(); i++)
	if (pairwise_matches[i].src_img_idx != pairwise_matches[i].dst_img_idx)
	    Save_match_data(grupo, pairwise_matches[i], Save);
    
    vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
    int VDis = Distancia (contador, indices, pairwise_matches, features, images, full_img_sizes); 
    cout << "Finaliza Match" << images.size() << endl;
    
    num_images = static_cast<int>(imgs.size()); 
    if (num_images < 2 || VDis == 2)
    {
        LOGLN("Need more images");
	cout << "No hay suficiente conincidencia entre imagenes" << endl;
        return 2;
    }

    float warped_image_scale;
    vector<CameraParams> cameras;
    vector<double> Distancia_f(num_images*2), Distancia_fB(num_images*2);

    Cambio_DF (pairwise_matches, Distancia_f);
    HomographyBasedEstimator estimator;
    estimator(features, pairwise_matches, cameras);
    Distancia_fB = Distancia_f;
    sort(Distancia_f.begin(), Distancia_f.end());
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        Mat R;
        cameras[i].R.convertTo(R, CV_32F);
        cameras[i].R = R;
	//cameras[i].focal = (Distancia_f[Distancia_f.size()/2 - 1] + Distancia_f[Distancia_f.size()/2]) * 0.5;
        LOGLN("Initial intrinsics #" << indices[i]+1 << ":\n" << cameras[i].K());
    }
    Save_camera_data(grupo, cameras, Distancia_fB);
    Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
    data_initialization (refine_mask);
    adjuster->setConfThresh(conf_thresh);
    adjuster->setRefinementMask(refine_mask);
    (*adjuster)(features, pairwise_matches, cameras);

    bool flagth_length_focal = false;
    vector<double> focals;
    for (size_t i = 0; i < cameras.size(); ++i)
    {
        LOGLN("Camera #" << indices[i]+1 << ":\n" << cameras[i].K());
        focals.push_back(cameras[i].focal);
	if (cameras[i].focal != cameras[i].focal)
	    flagth_length_focal = true;
    }
    Save_camera_data(grupo, cameras, Distancia_fB);
    sort(focals.begin(), focals.end());
    if (focals.size() % 2 == 1)
        warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
    else
        warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

    num_images = static_cast<int>(imgs.size());
    if (flagth_length_focal == true)
    {
        LOGLN("Need more images");
	cout << "Fallo proyeccion" << endl;	
        return 2;
    }
    grupo++;
    LOGLN("Warping images (auxiliary)... ");

    vector<Point> corners(num_images);
    vector<Mat> masks_warped(num_images);
    vector<Mat> images_warped(num_images);
    vector<Size> sizes(num_images);
    vector<Mat> masks(num_images);

    for (int i = 0; i < num_images; ++i)
    {
        masks[i].create(images[i].size(), CV_8U);
        masks[i].setTo(Scalar::all(255));
    }

    Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

    for (int i = 0; i < num_images; ++i)
    {
        Mat_<float> K;
        cameras[i].K().convertTo(K, CV_32F);
        float swa = (float)seam_work_aspect;
        K(0,0) *= swa; K(0,2) *= swa;
        K(1,1) *= swa; K(1,2) *= swa;
        corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LANCZOS4, BORDER_REFLECT, images_warped[i]);	
        sizes[i] = images_warped[i].size();
        warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
    }
    cout << "Finaliza proyeccion a segundo Plano" << endl;

    vector<Mat> images_warped_f(num_images);
    for (int i = 0; i < num_images; ++i)
        images_warped[i].convertTo(images_warped_f[i], CV_32F);

    seam_finder->find(images_warped_f, corners, masks_warped);

    images_warped.clear();
    images_warped_f.clear();
    masks.clear();
    LOGLN("Compositing...");

    Mat img_warped, img_warped_s;
    Mat dilated_mask, seam_mask, mask, mask_warped;
    Ptr<Blender> blender;
    double compose_work_aspect = 1;

    for (int img_idx = 0; img_idx < num_images; ++img_idx)
    {	
        LOGLN("Compositing image #" << indices[img_idx]+1);
	full_img = imgs[img_idx].clone();
        if (!is_compose_scale_set)
        {
            if (compose_megapix > 0)
                compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
            is_compose_scale_set = true;

            compose_work_aspect = compose_scale / work_scale;

            warped_image_scale *= static_cast<float>(compose_work_aspect);
            warper = warper_creator->create(warped_image_scale);

            for (int i = 0; i < num_images; ++i)
            {
                cameras[i].focal *= compose_work_aspect;
                cameras[i].ppx *= compose_work_aspect;
                cameras[i].ppy *= compose_work_aspect;

                Size sz = full_img_sizes[i];
                if (std::abs(compose_scale - 1) > 1e-1)
                {
                    sz.width = cvRound(full_img_sizes[i].width * compose_scale);
                    sz.height = cvRound(full_img_sizes[i].height * compose_scale);
                }

                Mat K;
                cameras[i].K().convertTo(K, CV_32F);
                Rect roi = warper->warpRoi(sz, K, cameras[i].R);
                corners[i] = roi.tl();
                sizes[i] = roi.size();
            }
        }
        if (abs(compose_scale - 1) > 1e-1)
            resize(full_img, img, Size(), compose_scale, compose_scale);
        else
            img = full_img;
        full_img.release();
        Size img_size = img.size();

        Mat K;
        cameras[img_idx].K().convertTo(K, CV_32F);
        warper->warp(img, K, cameras[img_idx].R, INTER_LANCZOS4, BORDER_REFLECT, img_warped);
        mask.create(img_size, CV_8U);
        mask.setTo(Scalar::all(255));
        warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);
        img_warped.convertTo(img_warped, CV_8U);
        img_warped.convertTo(img_warped_s, CV_16S);
        img_warped.release();
        img.release();
        mask.release();

        dilate(masks_warped[img_idx], dilated_mask, Mat());

        resize(dilated_mask, seam_mask, mask_warped.size());
 
        mask_warped = seam_mask & mask_warped;
        if (blender.empty())
        {
            blender = Blender::createDefault(blend_type, try_gpu);

            Size dst_sz = resultRoi(corners, sizes).size();
            float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
            if (blend_width < 1.f)
                blender = Blender::createDefault(Blender::NO, try_gpu);
            else if (blend_type == Blender::MULTI_BAND)
            {
                MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(static_cast<Blender*>(blender));

                mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
                LOGLN("Multi-band blender, number of bands: " << mb->numBands());
            }
            else if (blend_type == Blender::FEATHER)
            {
                FeatherBlender* fb = dynamic_cast<FeatherBlender*>(static_cast<Blender*>(blender));
                fb->setSharpness(1.f/blend_width);
                LOGLN("Feather blender, sharpness: " << fb->sharpness());
            }
            blender->prepare(corners, sizes);
        }
        blender->feed(img_warped_s, mask_warped, corners[img_idx]);
    }
    Mat result, result_mask;
	
    blender->blend(result, result_mask);

    pano = result.clone();
    cout << "Termino Composicion" << endl;
    LOGLN("Termino Funcion, time: " << ((getTickCount() - t) / getTickFrequency()) << " sec");
    return 1;
}

int main( int argc, char** argv )
{
    ros::init(argc,argv,"stitching");
    ros::NodeHandle camImage;

    ros::Subscriber img = camImage.subscribe("/stitching/image", 5, Image_Upload);
    ros::Rate loop_rate(2);

    time_t rawtime;
    struct tm * timeinfo;
    int status, NPanoramica = 1, Nimg = 1;
    time ( &rawtime );
    timeinfo = localtime (&rawtime); 
    char save[150], save2[150];  
    string result_name, image_name;

    strftime (Directorio_Images,150,"/home/caro/fuerte_workspace/sandbox/stitching_panoramic/AR_Drone/complete_mosaic/Images/%F-%R",timeinfo);	
    status = mkdir(Directorio_Images, S_IRWXU |S_IRWXG | S_IROTH | S_IXOTH);
    strftime (Directorio_Result,150,"/home/caro/fuerte_workspace/sandbox/stitching_panoramic/AR_Drone/complete_mosaic/Result/%F-%R",timeinfo);
    status = mkdir(Directorio_Result, S_IRWXU |S_IRWXG | S_IROTH | S_IXOTH);
    strcpy(save,Directorio_Result);
    strcpy(&save[ strlen(Directorio_Result) ], "/keypoints");
    status = mkdir(save, S_IRWXU |S_IRWXG | S_IROTH | S_IXOTH);
    strcpy(save2,Directorio_Result);
    strcpy(&save2[ strlen(Directorio_Result) ], "/match");
    status = mkdir(save2, S_IRWXU |S_IRWXG | S_IROTH | S_IXOTH);

for(;;)
{
    while(imageready!=1)
    { 
	//ROS_INFO(" Waitting for image to be ready\n");
	ros::spinOnce();
	loop_rate.sleep();
    }

    Mat IM = cv_ptr->image;
    imgs.push_back(IM);
    ostringstream os;
    os << Directorio_Images << "/IMG_" << Nimg << ".jpg"; 
    image_name = os.str();
    imwrite(image_name, IM);
    Nimg++;
 
    if (imgs.size() == 3)
    {
	contador = contador + imgs.size();
	int hoo = panoramic();
	imgs.clear();
	if (hoo == 1)
	{
    	    ostringstream os;
    	    os << Directorio_Result << "/result" << NPanoramica << ".jpg"; 
    	    result_name = os.str();

	    imgs.push_back(pano);
	    imwrite(result_name, pano);
	    NPanoramica++;
	    imshow("Stitching", pano);
	    waitKey(0);
	    
	}
	//contador++;
    }
    imageready=0;

    ros::spinOnce();
    loop_rate.sleep();
}
}
